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
