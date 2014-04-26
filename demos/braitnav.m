
% Copyright (C) 1993-2014, by Peter I. Corke
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


%%begin
% A Braitenberg vehicle is a simple reactive machine, that is, it exhibits useful
% behaviours based on how its sensors are connected to its actuators.

sl_braitenberg

% In this example the robot moves to the maximum of some scalar field which is
% measured by two separate sensors on the robot.  The sensors are simulated by
%the code

type sensorfield

% We will run the simulator and see that it moves toward the maxima
sim('sl_braitenberg');
