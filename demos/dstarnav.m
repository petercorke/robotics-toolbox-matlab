
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

% Our robot will operate within a grid world that contains free
% space where it can drive and obstacles.  We load a map of the world
load map1
% which loads a variable called map
about map

% The elements of the obstacle map are converted into traversibility values, an
% obstacle has an infinite traversibility while free space has a nominal
% travesibility of 1.

% Now we create an instance of a robot with the D* navigation algorithm
dstar = Dstar(map, 'quiet');

% Now we define the goal and start coordinates
goal = [50,30];
start = [20, 10];

% then ask the robot to plan a path to goal (it will take few seconds)
tic; dstar.plan(goal); toc

% Now we can display the obstacles and the cost to reach the goal from every
% point in the world
dstar.plot()
% where the cost scale is shown by the bar to the right.

% Now we can execute the planned path, it will be animated with green dots
dstar.path(start)

% Now lets change the difficulty of some of the terrain, make it more costly
% to travese
for r=78:85
    for c=12:45
        dstar.modify_cost([c,r], 2); 
    end
end

% Now we can replan, but D* does this in an incremental way which is faster than
% recomputing the whole plan from scratch
tic; dstar.plan(); toc
% that took less time than before...

% and the best path is now
dstar.path(start)
% we can see that the path has avoided the high cost region we added
