% Copyright (C) 1993-2013, by Peter I. Corke
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

% Our SLAM system requires a number of components:
% * a vehicle
% * a map that defines the positions of some known landmarks in the world
% * a sensor, a range-bearing sensor in this case
% * a SLAM filter

% Creating the vehicle.  First we define the covariance of the vehicles's odometry
% which reports distance travelled and change in heading angle

V = diag([0.005, 0.5*pi/180].^2);

% then use this to create an instance of a Vehicle class
veh = Vehicle(V);

% and then add a "driver" to move it between random waypoints in a square
% region with dimensions from -10 to +10

veh.add_driver( RandomPath(10) );

% Creating the map.  The map covers a square region with dimensions from 
% -10 to +10 and contains 20 randomly placed landmarks
map = Map(20, 10);

% Creating the sensor.  We firstly define the covariance of the sensor measurements
% which report distance and bearing angle
W = diag([0.1, 1*pi/180].^2);

% and then use this to create an instance of the Sensor class.
sensor = RangeBearingSensor(veh, map, W, 'animate');
% Note that the sensor is mounted on the moving robot and observes the features
% in the world so it is connected to the already created Vehicle and Map objects.

% Create the filter.  First we need to determine the initial covariance of the
% vehicle, this is our uncertainty about its pose (x, y, theta)
P0 = diag([0.005, 0.005, 0.001].^2);

% Now we create an instance of the EKF filter class
ekf = EKF(veh, V, P0, sensor, W, []);
% and connect it to the vehicle and the sensor and give estimates of the vehicle
% and sensor covariance (we never know this is practice).

% Now we will run the filter for 1000 time steps.  At each step the vehicle
% moves, reports its odometry and the sensor measurements and the filter updates
% its estimate of the vehicle's pose
ekf.run(1000);
% all the results of the simulation are stored within the EKF object

% First let's plot the map
clf; map.plot()
% and then overlay the path actually taken by the vehicle
veh.plot_xy('b');
% and then overlay the path estimated by the filter
ekf.plot_xy('r');
% which we see are pretty close

% Now let's plot the error in estimating the pose
ekf.plot_error()
% and this is overlaid with the estimated covariance of the error.

% Remember that the SLAM filter has not only estimated the robot's pose, it has
% simultaneously estimated the positions of the landmarks as well.  How well did it
% do at that task?  We will show the landmarks in the map again
map.plot();
% and this time overlay the estimated landmark (with a +) and the 3sigma 
% uncertainty bounds as green ellipses
ekf.plot_map(3,'g');
