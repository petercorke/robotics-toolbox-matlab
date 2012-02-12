%RangeBearingSensor Range and bearing sensor class
%
% A concrete subclass of Sensor that implements a range and bearing angle
% sensor that provides robot-centric measurements of the world. To enable
% this it has references to a map of the world (Map object) and a robot
% moving through the world (Vehicle object).
%
% Methods::
%
% reading   return a random range/bearing observation
% h         return the observation for vehicle state xv and feature xf
% Hx        return a Jacobian matrix dh/dxv 
% Hxf       return a Jacobian matrix dh/dxf 
% Hw        return a Jacobian matrix dh/dw
%
% g         return feature positin given vehicle pose and observation
% Gx        return a Jacobian matrix dg/dxv 
% Gz        return a Jacobian matrix dg/dz
%
% Properties (read/write)::
% R            measurement covariance matrix
% interval     valid measurements returned every interval'th call to reading()
%
%
% Reference::
%
%   Robotics, Vision & Control,
%   Peter Corke,
%   Springer 2011
%
% See also Sensor, Vehicle, Map, EKF.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

classdef RangeBearingSensor < Sensor

    properties
        R           % measurment covariance
        interval    % measurement return subsample factor
        r_range     % range limits
        theta_range % angle limits

        randstream  % random stream just for Sensors
    end

    properties (SetAccess = private)
        count       % number of reading()s
    end

    methods

        function s = RangeBearingSensor(robot, map, R)
            %RangeBearingSensor.RangeBearingSensor Range and bearing sensor constructor
            %
            % S = RangeBearingSensor(VEHICLE, MAP, R, OPTIONS) is a range
            % and bearing angle sensor mounted on the Vehicle object
            % VEHICLE and observing the landmark map MAP.  The sensor
            % covariance is R (2x2) representing range and bearing
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
            % See also Sensor, Vehicle, Map, EKF.


            % call the superclass constructor
            s = s@Sensor(robot, map);

            s.randstream = RandStream.create('mt19937ar');

            s.R = R;
            s.r_range = [];
            s.theta_range = [];
            s.interval = 1;
            s.count = 0;
        end

        function k = selectFeature(s)
            k = s.randstream.randi(s.map.nfeatures);
        end

        % return sensor reading
        function [z,jf] = reading(s)
            %RangeBearingSensor.h Landmark range and bearing
            %
            % S.reading() is a range/bearing observation [Z,J] where
            % Z=[R,THETA] is range and bearing with additive Gaussian noise
            % of covariance R. J is the index of the map feature that was
            % observed. If no valid measurement, ie. no features within
            % range, interval subsampling enabled or simulated failure the
            % return is Z=[] and J=NaN.
            %
            % See also RangeBearingSensor.h.
            
            % model a sensor that emits readings every interval samples
            s.count = s.count + 1;
            if mod(s.count, s.interval) ~= 0
                z = [];
                jf = NaN;
                return;
            end

            % randomly choose the feature
            jf = s.selectFeature();

            % compute the range and bearing from robot to feature
            %z = s.h(s.robot.x, jf) + sqrt(s.R)*s.randstream.randn(2,1);
            %z(2) = angdiff(z(2));
            z = s.h(s.robot.x, jf);

            if s.verbose
                fprintf('Sensor:: feature %d: %.1f %.1f\n', k, z);
            end

            % if range and bearing angle limits are in place, enforce them
            %  return jf=NaN in this case
            if ~isempty(s.r_range)
                if z(1) < s.r_range(1) || z(1)r > s.r_range(2)
                    z = [];
                    jf = NaN;
                    return;
                end
            end
            if ~isempty(s.theta_range)
                if z(2) < s.theta_range(1) || z(2) > s.theta_range(2)
                    z = [];
                    jf = NaN;
                    return;
                end
            end
        end


        function z = h(s, xv, jf)
            %RangeBearingSensor.h Landmark range and bearing
            %
            % Z = S.h(XV, J) is range and bearing from vehicle at XV to map
            % feature J.  Z = [R,theta]
            %
            % Z = S.h(XV, XF) as above but compute range and bearing to a
            % feature at coordinate XF.
            %
            % See also RangeBearingSensor.Hx, RangeBearingSensor.Hw, RangeBearingSensor.Hxf.
            if length(jf) == 1
                xf = s.map.map(:,jf);
            else
                xf = jf;
            end
            dx = xf(1) - xv(1); dy = xf(2) - xv(2);

            z = zeros(2,1);
            z(1) = sqrt(dx^2 + dy^2);       % range measurement
            z(2) = atan2(dy, dx) - xv(3);   % bearing measurement
        end

        function J = Hx(s, xv, jf)
            %RangeBearingSensor.Hx Jacobian dh/dxv
            %
            % J = S.Hx(XV, J) returns the Jacobian dh/dxv at the vehicle
            % state XV, for map feature J.  J is 2x3.
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
            Delta = xf - xv(1:2);
            r = norm(Delta);
            J = [
                -Delta(1)/r,    -Delta(2)/r,        0
                Delta(2)/(r^2),  -Delta(1)/(r^2),   -1
            ];
        end

        function J = Hxf(s, xv, jf)
            %RangeBearingSensor.Hxf Jacobian dh/dxf
            %
            % J = S.Hxf(XV, J) returns the Jacobian dh/dxv at the vehicle
            % state XV, for map feature J.  J is 2x2.
            %
            % J = S.Hxf(XV, XF) as above but for a feature at coordinate XF.
            %
            % See also RangeBearingSensor.h.
            if length(jf) == 1
                xf = s.map.map(:,jf);
            else
                xf = jf;
            end
            Delta = xf - xv(1:2);
            r = norm(Delta);
            J = [
                Delta(1)/r,         Delta(2)/r
                -Delta(2)/(r^2),    Delta(1)/(r^2)
            ];
        end

        function J = Hw(s, xv, jf)
            %RangeBearingSensor.Hx Jacobian dh/dv
            %
            % J = S.Hw(XV, J) returns the Jacobian dh/dv at the vehicle
            % state XV, for map feature J.  J is 2x2.
            %
            % See also RangeBearingSensor.h.
            J = eye(2,2);
        end

        function xf = g(s, xv, z)
            %RangeBearingSensor.g Compute landmark location
            %
            % P = S.g(XV, Z) is the world coordinate of feature given
            % observation Z and vehicle state XV.
            %
            % See also RangeBearingSensor.Gx, RangeBearingSensor.Gz.
            range = z(1);
            bearing = z(2) + xv(3);

            xf = xv(1:2) + [range*cos(bearing); range*sin(bearing)];
        end

        function J = Gx(s, xv, z)
            %RangeBearingSensor.Gxv Jacobian dg/dx
            %
            % J = S.Gx(XV, Z) returns the Jacobian dg/dxv at the vehicle state XV, for
            % measurement Z.  J is 2x3.
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
            % J = S.Gz(XV, Z) returns the Jacobian dg/dz at the vehicle state XV, for
            % measurement Z.  J is 2x2.
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

    end % method
end % classdef
