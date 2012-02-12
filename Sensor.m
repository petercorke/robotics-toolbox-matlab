%Sensor Sensor superclass
%
% An abstact superclass to represent robot navigation sensors.
%
% S = Sensor(VEHICLE, MAP, R) is an instance of the Sensor object that
% references the VEHICLE on which the sensor is mounted, the MAP of
% landmarks that it is observing, and the sensor covariance matrix R.
%
% Methods::
%   display     print the parameters in human readable form
%   char        convert the parameters to a human readable string
%
% Properties::
% robot   The Vehicle object on which the sensor is mounted
% map     The Map object representing the landmarks around the robot
%
% Reference::
%
%   Robotics, Vision & Control,
%   Peter Corke,
%   Springer 2011
%
% See also EKF, Vehicle, Map.

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

classdef Sensor < handle

    properties
        robot
        map

        verbose
    end

    methods

        function s = Sensor(robot, map, R)
            %Sensor.Sensor Sensor object constructor
            %
            % S = Sensor(VEHICLE, MAP) is a sensor mounted on the Vehicle object
            % VEHICLE and observing the landmark map MAP.
            
            s.robot = robot;
            s.map = map;
            s.verbose = false;
        end

%         function k = selectFeature(s)
%             k = randi(s.map.nfeatures);
%             % could add some simulated sensor failure mode in here by returning NaN
%         end

%     % return sensor reading
%     function [z,jf] = reading(s)
%         s.count = s.count + 1;
%         if s.count < s.interval
%             z = [];
%             jf = NaN;
%             return;
%         end
%         s.count = 0;
% %%            if r < s.r_range(0) || r > s.r_range(1)
% %%                z = [];
% %%                return;
% %%            end
% %%            if theta < s.theta_range(0) || theta > s.theta_range(1)
% %%                z = [];
% %%            return;
%         jf = s.selectFeature();
%         if ~isnan(jf)
%             z = s.h(s.robot.x, jf) + sqrt(s.R)*randn(2,1);
%             z(2) = angdiff(z(2));
%         end
%         if s.verbose
%             fprintf('Sensor:: feature %d: %.1f %.1f\n', k, z);
%         end
%     end

        function display(s)
            %Sensor.display Display status of sensor object
            %
            % S.display() display the state of the sensor object in
            % human-readable form.
            %
            % Notes::
            % - this method is invoked implicitly at the command line when the result
            %   of an expression is a Sensor object and the command has no trailing
            %   semicolon.
            %
            % See also Sensor.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(s) );
        end % display()

        function str = char(s)
            %Sensor.char Convert sensor parameters to a string
            %
            % s = S.char() is a string showing sensor parameters in
            % a compact human readable format.
            str = 'Sensor object';
            str = strvcat(str, sprintf('  %d features', s.map.nfeatures));
            str = strvcat(str, sprintf('  dimension %.1f', s.map.dim));
        end

    end % method
end % classdef
