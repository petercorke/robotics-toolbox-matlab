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
