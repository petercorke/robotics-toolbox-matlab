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

% Jacobian and differential motion demonstration
%
% A differential motion can be represented by a 6-element vector with elements
% [dx dy dz drx dry drz]
%
% where the first 3 elements are a differential translation, and the last 3 
% are a differential rotation.  When dealing with infinitisimal rotations, 
% the order becomes unimportant.  The differential motion could be written 
% in terms of compounded transforms
%
% transl(dx,dy,dz) * trotx(drx) * troty(dry) * trotz(drz)
%
% but a more direct approach is to use the function diff2tr()

D = [.1 .2 0 -.2 .1 .1]';
delta2tr(D)

% More commonly it is useful to know how a differential motion in one 
% coordinate frame appears in another frame.  If the second frame is 
% represented by the transform

T = transl(100, 200, 300) * troty(pi/8) * trotz(-pi/4);

% then the differential motion in the second frame would be given by

DT = tr2jac(T) * D;
DT'

% tr2jac() has computed a 6x6 Jacobian matrix which transforms the differential 
% changes from the first frame to the next.

%------
% The manipulator's Jacobian matrix relates differential joint coordinate 
% motion to differential Cartesian motion;
%
% 	dX = J(q) dQ
%
% For an n-joint manipulator the manipulator Jacobian is a 6 x n matrix and
% is used is many manipulator control schemes.  

% We will work with a model of the Puma 560 robot
mdl_puma560

% Two Jacobians are frequently used, which express the Cartesian velocity in
% the world coordinate frame,
%
% We will first choose a particular joint angle configuration for the robot

q = [0.1 0.75 -2.25 0 .75 0]

% and then compute the Jacobian in the world coordinate frame
J = p560.jacob0(q)
% which we can see is 6x6 (since the robot has 6 joints)

% Alternatively the Jacobian can be expressed in the T6 coordinate frame

J = p560.jacobn(q)
% Note the top right 3x3 block is all zero.  This indicates, correctly, that
% motion of joints 4-6 does not cause any translational motion of the robot's
% end-effector.

% Many control schemes require the inverse of the Jacobian.  The Jacobian
% in this example is not singular

det(J)

% and may be inverted
Ji = inv(J)

% A classic control technique is Whitney's resolved rate motion control
%
% dQ/dt = J(q)^-1 dX/dt
%
% where dX/dt is the desired Cartesian velocity, and dQ/dt is the required
% joint velocity to achieve this.

vel = [1 0 0 0 0 0]'; % translational motion in the X direction
qvel = Ji * vel;
qvel'

% This is an alternative strategy to computing a Cartesian trajectory 
% and solving the inverse kinematics.  However like that other scheme, this
% strategy also runs into difficulty at a manipulator singularity where
% the Jacobian is singular.

% As already stated this Jacobian relates joint velocity to end-effector 
% velocity expressed in the end-effector reference frame.  We may wish 
% instead to specify the velocity in base or world coordinates.
%
% We have already seen how differential motions in one frame can be translated 
% to another.  Consider the velocity as a differential in the world frame, that
% is, d0X.  We can write
% 	d6X = Jac(T6) d0X

T6 = p560.fkine(q); % compute the end-effector transform
d6X = tr2jac(T6) * vel; % translate world frame velocity to T6 frame
qvel = Ji * d6X; % compute required joint velocity as before
qvel'

% Note that this value of joint velocity is quite different to that calculated
% above, which was for motion in the T6 X-axis direction.

% At a manipulator singularity or degeneracy the Jacobian becomes singular.
% At the Puma's `ready' position for instance, two of the wrist joints are
% aligned resulting in the loss of one degree of freedom.  This is revealed by
% the rank of the Jacobian
rank( p560.jacobn(qr) )

% and the singular values are
svd( jacobn(p560, qr) )

% When not actually at a singularity the Jacobian can provide information 
% about how `well-conditioned' the manipulator is for making certain motions,
% and is referred to as `manipulability'.
%
% A number of scalar manipulability measures have been proposed.  One by
% Yoshikawa

p560.maniplty(q, 'yoshikawa')

% is based purely on kinematic parameters of the manipulator.
%
% Another by Asada takes into account the inertia of the manipulator which 
% affects the acceleration achievable in different directions.  This measure 
% varies from 0 to 1, where 1 indicates uniformity of acceleration in all 
% directions

p560.maniplty(q, 'asada')

% Both of these measures would indicate that this particular pose is not well
% conditioned.
