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

% A serial link manipulator comprises a series of links.  Each link is described
% by four Denavit-Hartenberg parameters.
%
% Let's define a simple 2 link manipulator.  The first link is

L1 = Link('d', 0, 'a', 1, 'alpha', pi/2)

% The Link object we created has a number of properties
L1.a
L1.d

% and we determine that it is a revolute joint
L1.isrevolute

% For a given joint angle, say q=0.2 rad, we can determine the link transform
% matrix
L1.A(0.2)

% The second link is
L2 = Link('d', 0, 'a', 1, 'alpha', 0)

% Now we need to join these into a serial-link robot manipulator

bot = SerialLink([L1 L2], 'name', 'my robot')
% The displayed robot object shows a lot of details.  It also has a number of
% properties such as the number of joints
bot.n

% Given the joint angles q1 = 0.1 and q2 = 0.2 we can determine the pose of the
% robot's end-effector

bot.fkine([0.1 0.2])
% which is referred to as the forward kinematics of the robot.  This, and the
% inverse kinematics are covered in separate demos.

% Finally we can draw a stick figure of our robot

bot.plot([0.1 0.2])
