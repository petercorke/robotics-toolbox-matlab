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

% Frequently we want to define a smooth sequence of positions (or poses) from
% one point to another.  First consider the 1-dimensional case.
%
% We define the start and end position

p0 = -1;
p1 = 2;

% and a smooth path from p0 to p1 in 50 time steps is given by

p = tpoly(p0, p1, 50);
about p
% which we see has 50 rows.  We can plot this 

plot(p)

% and see that it does indeed move smoothly from p0 to p1 and that the initial
% and final derivative (and second derivative) is zero.

% We can also get the velocity and acceleration

[p,pd,pdd] = tpoly(p0, p1, 50);
subplot(3,1,1); plot(p); xlabel('Time'); ylabel('p');
subplot(3,1,2); plot(pd); xlabel('Time'); ylabel('pd');
subplot(3,1,3); plot(pdd); xlabel('Time'); ylabel('pdd');

% This path is a 5th order polynomial and it suffers from the disadvantage that
% the velocity is mostly below the maximum possible value.  An alternative is

[p,pd,pdd] = lspb(p0, p1, 50);
subplot(3,1,1); plot(p); xlabel('Time'); ylabel('p');
subplot(3,1,2); plot(pd); xlabel('Time'); ylabel('pd');
subplot(3,1,3); plot(pdd); xlabel('Time'); ylabel('pdd');
% which we see has a trapezoidal velocity profile.

% Frequently the start and end values are vectors, not scalars, perhaps a 3D
% position or Euler angles.  In this case we apply the scalar trajectory function
% to a vector with

p = mtraj(@tpoly, [0 1 2], [2 1 0], 50);
about p
% and p again has one row per time step, and one column per vector dimension

clf; plot(p)

%---
% Finally, we may wish to interpolate poses.  We will define a start and end pose

T0 = transl(0.4, 0.2, 0) * trotx(pi);
T1 = transl(-0.4, -0.2, 0.3) * troty(pi/2) * trotz(-pi/2);

% and a smooth sequence between them in 50 steps is

T = ctraj(T0, T1, 50);
about T
% which is a 4x4x50 matrix.  The first pose is

T(:,:,1)

% and the 10th pose is

T(:,:,10)

% We can plot the motion of this coordinate frame by

clf; tranimate(T)
