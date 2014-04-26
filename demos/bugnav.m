
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

% Our bug style robot will operate within a grid world that contains free
% space where it can drive and obstacles.  We load a map of the world
load map1
% which loads a variable called map
about map
% and the cells contain zero if it is free space (driveable) and one if it is an
% obstacle.

% Now we create an instance of a robot with the bug navigation algorithm
bug = Bug2(map)

% and display its grid world
bug.plot()
% where obstacles are marked in red.

% Now we define the goal and start coordinates
bug.goal = [50,30];
start = [20, 10];

% then ask the robot to find the path, it will be animated with green dots
bug.path(start);
