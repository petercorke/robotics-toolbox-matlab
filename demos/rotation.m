
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

% In the field of robotics there are many possible ways of representing 
% orientations of which the most common are: 
% - orthonormal rotation matrices (3x3),
% - three angles (1x3), and
% - quaternions.

% A rotation of pi/2 about the x-axis can be represented as an orthonormal rotation
% matrix

R = rotx(pi/2)
% which we can see is a 3x3 matrix.

% Such a matrix has the property that it's columns (and rows) are sets of orthogonal
% unit vectors.  The determinant of such a matrix is always 1

det(R)

% Let's create a more complex rotation

R = rotx(30, 'deg') * roty(50, 'deg') * rotz(10, 'deg')
% where this time we have specified the rotation angle in degrees.

% Any rotation can be expressed in terms of a single rotation about some axis
% in space

[theta,vec] = tr2angvec(R)
% where theta is the angle (in radians) and vec is unit vector representing the
% direction of the rotation axis.

% Commonly rotations are represented by Euler angles

eul = tr2eul(R)
% which are three angles such that R = rotz(a)*roty(b)*rotz(c), ie. the rotations
% required about the Z, then then the Y, then the Z axis.

% Rotations are also commonly represented by roll-pitch-yaw  angles

rpy = tr2rpy(R)
% which are three angles such that R = rotx(r)*roty(p)*rotz(y), ie. the rotations
% required about the X, then then the Y, then the Z axis.

% We can investigate the effects of rotations about different axes
% using this GUI based demonstration.  The menu buttons allow the rotation
% axes to be varied
%  *** close the window when you are done.
tripleangle('rpy', 'wait')

% The final useful form is the quaternion which comprises 4 numbers.  We can create
% a quaternion from an orthonormal matrix

q = Quaternion(R)
% where we can see that it comprises a scalar part and a vector part.  To convert back

q.R
% which is the same of the value of R we determined above.

% Quaternions are a class and the orientations they represent can be compounded, just
% as we do with rotation matrices by multiplication.

% First we create two quaternions

q1 = Quaternion( rotx(pi/2) )
q2 = Quaternion( roty(pi/2) )

% then the rotation of q1 followed by q2 is simply

q1 * q2

% We can also take the inverse of a Quaternion

q1 * inv(q1)
% which results in a null rotation (zero vector part)
