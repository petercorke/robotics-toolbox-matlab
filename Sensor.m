%Sensor Sensor superclass
%
% An abstact superclass to represent robot navigation sensors.
%
% Methods::
%   display     print the parameters in human readable form
%   char        convert to string
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
    % TODO, pose option, wrt vehicle

    properties
        robot
        map

        verbose
        
        ls
    end

    methods

        function s = Sensor(robot, map, varargin)
        %Sensor.Sensor Sensor object constructor
        %
        % S = Sensor(VEHICLE, MAP) is a sensor mounted on the Vehicle object
        % VEHICLE and observing the landmark map MAP.
        % S = Sensor(VEHICLE, MAP, R) is an instance of the Sensor object mounted
        % on a vehicle represented by the object VEHICLE and observing features in the
        % world represented by the object MAP.
            s.robot = robot;
            s.map = map;
            s.verbose = false;
        end
        
        function plot(s, jf)
            if ~isempty(s.ls)
                xi = s.map.map(:,jf);
                h = plot([s.robot.x(1), xi(1)], [s.robot.x(2), xi(2)], s.ls);
                pause(0.3);
                delete(h);
                drawnow
            end
        end

        function display(s)
            %Sensor.display Display status of sensor object
            %
            % S.display() displays the state of the sensor object in
            % human-readable form.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
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
            str = [class(s) ' sensor class:'];
            str = char(str, char(s.map));
        end

    end % method
end % classdef
