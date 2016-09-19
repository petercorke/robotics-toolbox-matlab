%RangeBearingSensor Range and bearing sensor class
%
% A concrete subclass of the Sensor class that implements a range and bearing
% angle sensor that provides robot-centric measurements of point features in 
% the world. To enable this it has references to a map of the world (PointMap object)
% and a robot moving through the world (Vehicle object).
%
% Methods::
%
% reading   range/bearing observation of random feature
% h         range/bearing observation of specific feature
% Hx        Jacobian matrix dh/dxv 
% Hxf       Jacobian matrix dh/dxf 
% Hw        Jacobian matrix dh/dw
%-
% g         feature position given vehicle pose and observation
% Gx        Jacobian matrix dg/dxv 
% Gz        Jacobian matrix dg/dz
%
% Properties (read/write)::
% W            measurement covariance matrix (2x2)
% interval     valid measurements returned every interval'th call to reading()
%
% Reference::
%
%   Robotics, Vision & Control, Chap 6,
%   Peter Corke,
%   Springer 2011
%
% See also Sensor, Vehicle, PointMap, EKF.


% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

classdef RangeBearingSensor < Sensor

    properties
        W           % measurment covariance
        r_range     % range limits
        theta_range % angle limits

        randstream  % random stream just for Sensors
        
        landmarklog  % time history of observed landmarks
        
    end

    properties (SetAccess = private)
        count       % number of reading()s
    end

    methods

        function s = RangeBearingSensor(robot, map, varargin)
            %RangeBearingSensor.RangeBearingSensor Range and bearing sensor constructor
            %
            % S = RangeBearingSensor(VEHICLE, MAP, W, OPTIONS) is an object representing
            % a range and bearing angle sensor mounted on the Vehicle object
            % VEHICLE and observing an environment of known landmarks represented by the
            % map object MAP.  The sensor covariance is W (2x2) representing range and bearing
            % covariance.
            %
            % Options::
            % 'range', xmax            maximum range of sensor
            % 'range', [xmin xmax]     minimum and maximum range of sensor
            % 'angle', TH              detection for angles betwen -TH to +TH
            % 'angle', [THMIN THMAX]   detection for angles betwen THMIN
            %                          and THMAX
            % 'skip', I                return a valid reading on every I'th
            %                          call
            % 'fail', [TMIN TMAX]      sensor simulates failure between 
            %                          timesteps TMIN and TMAX
            %
            % See also options for Sensor constructor.
            %
            % See also Sensor.Sensor, Vehicle, PointMap, EKF.


            % call the superclass constructor
            s = s@Sensor(robot, map, varargin{:});

            s.randstream = RandStream.create('mt19937ar');

            opt.range = [];
            opt.angle = [];
            opt.covar = zeros(2,2);

            [opt,args] = tb_optparse(opt, varargin);

            s.W = opt.covar;
            if ~isempty(opt.range)
                if length(opt.range) == 1
                    s.r_range = [0 opt.range];
                elseif length(opt.range) == 2
                    s.r_range = opt.range;
                end
            end
            if ~isempty(opt.angle)
                if length(opt.angle) == 1
                    s.theta_range = [-opt.angle opt.angle];
                elseif length(opt.angle) == 2
                    s.theta_range = opt.angle;
                end
            end

            s.count = 0;
            s.verbose = opt.verbose;
        end

        function init(s)
            s.landmarklog = [];
        end
        
        function k = selectFeature(s)
            k = s.randstream.randi(s.map.nlandmarks);
        end

        function [z,jf] = reading(s)
            %RangeBearingSensor.h Landmark range and bearing
            %
            % [Z,K] = S.reading() is an observation of a random landmark where
            % Z=[R,THETA] is the range and bearing with additive Gaussian noise
            % of covariance W (property W). K is the index of 
            % the map feature that was observed. If no valid measurement, ie. no
            % features within range, interval subsampling enabled or simulated 
            % failure the return is Z=[] and K=0.
            %
            % See also RangeBearingSensor.h.
            
            % TODO probably should return K=0 to indicated invalid
            
            % model a sensor that emits readings every interval samples
            s.count = s.count + 1;

            % check conditions for NOT returning a value
            z = [];
            jf = 0;
            % sample interval
            if mod(s.count, s.interval) ~= 0
                return;
            end
            % simulated failure
            if ~isempty(s.fail) && (s.count >= s.fail(1)) && (s.count <= s.fail(2))
                return;
            end
            
            % create a polygon to indicate the active sensing area based on range+angle limits
            if ~isempty(s.theta_range) && ~isempty(s.r_range)
                h = findobj(gca, 'tag', 'sensor-area');
                if isempty(h)
                    
                    th=linspace(s.theta_range(1), s.theta_range(2), 20);
                    x = s.r_range(2) * cos(th);
                    y = s.r_range(2) * sin(th);
                    if s.r_range(1) > 0
                        th = flip(th);
                        x = [x s.r_range(1) * cos(th)];
                        y = [y s.r_range(1) * sin(th)];
                    else
                        x = [x 0];
                        y = [y 0];
                    end
                    % no sensor zone, create one
                    plot_poly([x; y], 'fill', 'r', 'alpha', 0.1, 'edge', 'none', 'animate', 'tag', 'sensor-area');
                else
                    %hg = get(h, 'Parent');
                    plot_poly(h, s.robot.x);
                    
                end
            end
            
            if ~isempty(s.r_range)  || ~isempty(s.theta_range)
                % if range and bearing angle limits are in place look for
                % any landmarks that match criteria
                
                % get range/bearing to all landmarks, one per row
                z = s.h(s.robot.x');
                jf = 1:numcols(s.map.map);
                
                if ~isempty(s.r_range)
                    % find all within range
                    k = find( z(:,1) >= s.r_range(1) & z(:,1) <= s.r_range(2) );
                    z = z(k,:);
                    jf = jf(k);
                end
                if ~isempty(s.theta_range)
                    % find all within angular range as well
                    k = find( z(:,2) >= s.theta_range(1) & z(:,2) <= s.theta_range(2) );
                    z = z(k,:);
                    jf = jf(k);
                end
                
                % deal with cases for 0 or > 1 features found
                if isempty(z)
                    % no landmarks found
                    jf = 0;
                elseif length(k) >= 1
                    % more than 1 in range, pick a random one
                    i = s.randstream.randi(length(k));
                    z = z(i,:);
                    jf = jf(i);
                end

            else
                % randomly choose the feature
                jf = s.selectFeature();
                
                % compute the range and bearing from robot to feature
                z = s.h(s.robot.x', jf);   
            end
            
            if s.verbose
                if isempty(z)
                    fprintf('Sensor:: no features\n');
                else
                    fprintf('Sensor:: feature %d: %.1f %.1f\n', jf, z);
                end
            end
            if s.animate
                s.plot(jf);
            end
            
            z = z';

            s.landmarklog = [s.landmarklog jf];
        end


        function z = h(s, xv, jf)
            %RangeBearingSensor.h Landmark range and bearing
            %
            % Z = S.h(XV, K) is a sensor observation (1x2), range and bearing, from vehicle at 
            % pose XV (1x3) to the K'th landmark.
            %
            % Z = S.h(XV, XF) as above but compute range and bearing to a landmark at coordinate XF.
            %
            % Z = s.h(XV) as above but computes range and bearing to all
            % map features.  Z has one row per landmark.
            %
            % Notes::
            % - Noise with covariance W (propertyW) is added to each row of Z.
            % - Supports vectorized operation where XV (Nx3) and Z (Nx2).
            %
            % See also RangeBearingSensor.Hx, RangeBearingSensor.Hw, RangeBearingSensor.Hxf.
            
            % get the landmarks, one per row
            if nargin < 3
                % s.h(XV)
                xlm = s.map.map';
            elseif length(jf) == 1
                % s.h(XV, JF)
                xlm = s.map.map(:,jf)';
            else
                % s.h(XV, XF)
                xlm = jf(:)';
            end
            
            % Straightforward code:
            %
            % dx = xf(1) - xv(1); dy = xf(2) - xv(2);
            %
            % z = zeros(2,1);
            % z(1) = sqrt(dx^2 + dy^2);       % range measurement
            % z(2) = atan2(dy, dx) - xv(3);   % bearing measurement
            %
            % Vectorized code:

            % compute range and bearing
            dx = xlm(:,1) - xv(:,1); dy = xlm(:,2) - xv(:,2);
            z = [sqrt(dx.^2 + dy.^2) angdiff(atan2(dy, dx), xv(:,3)) ];   % range & bearing measurement

            % add noise with covariance W
            z = z + s.randstream.randn(size(z)) * sqrtm(s.W) ;
        end

        function J = Hx(s, xv, jf)
            %RangeBearingSensor.Hx Jacobian dh/dxv
            %
            % J = S.Hx(XV, K) returns the Jacobian dh/dxv (2x3) at the vehicle
            % state XV (3x1) for map feature K.
            %
            % J = S.Hx(XV, XF) as above but for a feature at coordinate XF.
            %
            % See also RangeBearingSensor.h.
            if length(jf) == 1
                xf = s.map.map(:,jf);
            else
                xf = jf;
            end
            if isempty(xv)
                xv = s.robot.x;
            end
            Delta = xf - xv(1:2)';
            r = norm(Delta);
            J = [
                -Delta(1)/r,    -Delta(2)/r,        0
                Delta(2)/(r^2),  -Delta(1)/(r^2),   -1
            ];
        end

        function J = Hxf(s, xv, jf)
            %RangeBearingSensor.Hxf Jacobian dh/dxf
            %
            % J = S.Hxf(XV, K) is the Jacobian dh/dxv (2x2) at the vehicle
            % state XV (3x1) for map feature K.
            %
            % J = S.Hxf(XV, XF) as above but for a feature at coordinate XF (1x2).
            %
            % See also RangeBearingSensor.h.
            if length(jf) == 1
                xf = s.map.map(:,jf);
            else
                xf = jf;
            end
            Delta = xf - xv(1:2)';
            r = norm(Delta);
            J = [
                Delta(1)/r,         Delta(2)/r
                -Delta(2)/(r^2),    Delta(1)/(r^2)
            ];
        end

        function J = Hw(s, xv, jf)
            %RangeBearingSensor.Hx Jacobian dh/dv
            %
            % J = S.Hw(XV, K) is the Jacobian dh/dv (2x2) at the vehicle
            % state XV (3x1) for map feature K.
            %
            % See also RangeBearingSensor.h.
            J = eye(2,2);
        end

        function xf = g(s, xv, z)
            %RangeBearingSensor.g Compute landmark location
            %
            % P = S.g(XV, Z) is the world coordinate (1x2) of a feature given
            % the sensor observation Z (1x2) and vehicle state XV (3x1).
            %
            % See also RangeBearingSensor.Gx, RangeBearingSensor.Gz.

            range = z(1);
            bearing = z(2) + xv(3); % bearing angle in vehicle frame

            xf = [xv(1)+range*cos(bearing) xv(2)+range*sin(bearing)];
        end

        function J = Gx(s, xv, z)
            %RangeBearingSensor.Gxv Jacobian dg/dx
            %
            % J = S.Gx(XV, Z) is the Jacobian dg/dxv (2x3) at the vehicle state XV (3x1) for
            % sensor observation Z (2x1).
            %
            % See also RangeBearingSensor.g.
            theta = xv(3);
            r = z(1);
            bearing = z(2);
            J = [
                1,   0,   -r*sin(theta + bearing);
                0,   1,    r*cos(theta + bearing)
                ];
        end
        

        function J = Gz(s, xv, z)
            %RangeBearingSensor.Gz Jacobian dg/dz
            %
            % J = S.Gz(XV, Z) is the Jacobian dg/dz (2x2) at the vehicle state XV (3x1) for
            % sensor observation Z (2x1).
            %
            % See also RangeBearingSensor.g.
            theta = xv(3);
            r = z(1);
            bearing = z(2);
            J = [
                cos(theta + bearing),   -r*sin(theta + bearing);
                sin(theta + bearing),    r*cos(theta + bearing)
                ];
        end

        function str = char(s)
            str = char@Sensor(s);
            str = char(str, ['W = ', mat2str(s.W, 3)]);

            str = char(str, sprintf('interval %d samples', s.interval) );
            if ~isempty(s.r_range)
                str = char(str, sprintf('range: %g to %g', s.r_range) );
            end
            if ~isempty(s.theta_range)
                str = char(str, sprintf('angle: %g to %g', s.theta_range) );
            end
        end
        
    end % method
end % classdef
