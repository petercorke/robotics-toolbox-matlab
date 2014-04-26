
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

% A large number of Toolbox functions can operate on symbolic rather than
% numeric quantities.
%
% Consider the simple case of a rotation matrix

syms a
rotx(a)

% Or a more complex example for Euler angles

syms a b c
eul2r(a, b, c)

% Now let's consider a robot link with symbolic parameters

syms q A D alpha

L = Link('d', D, 'a', A, 'alpha', a, 'revolute');

% the link transform matrix for a joint angle q is then

L.A(q)

% Consider now a simple two-link robot, we load the model

mdl_twolink

% which has created a robot model in the workspace

twolink

% Now this is a numeric robot model, and we need to create a symbolic model

twolink_sym = twolink.sym()

% which appears very similar, however all the constants are now symbolic rather
% than numeric.

% Next define the two joint angles as symbolic variables

syms q1 q2

% and then the forward kinematics is

twolink_sym.fkine([q1, q2])
