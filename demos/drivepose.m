%%begin

% We will show how to make a mobile robot with "car-like" steering drive from
% one pose to another.

% The goal pose is
xg = [5 5 pi/2];

% and the starting pose is
x0 = [5 9 0]

% We use a Simulink model to represent the dynamics of the vehicle and to
% implement a pose controller
sl_drivepose

% We run the simulation
r = sim('sl_drivepose');
% and extract the trajectory of the robot
y = r.find('yout');

% which we plot
axis([0 10 0 10]); hold on; grid on
plot(y(:,1), y(:,2));
% and overlay the initial and final pose of the robot
plot_vehicle(x0, 'r'); plot_vehicle(xg, 'r');
% Note the complex path the robot had to follow, since it's motion is limited
% by the nature of the steering mechanism.
