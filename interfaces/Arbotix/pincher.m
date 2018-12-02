
% Copyright (C) 1993-2015, by Peter I. Corke
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
clear L
L(1) = Revolute('d', 40, 'alpha', -pi/2);
L(2) = Revolute('a', -105, 'alpha', pi, 'offset', pi/2);
L(3) = Revolute('a', -105);
L(4) = Revolute('a', -105);

% Note alpha_2 = pi, needed to account for rotation axes of joints 3 and 4 having
% opposite sign to joint 2.
%
% s='Rz(q1) Tz(L1) Ry(q2) Tz(L2) Ry(q3) Tz(L3) Ry(q4) Tz(L4)'
% DHFactor(s)


arb = Arbotix('port', '/dev/tty.usbserial-A800JDPN', 'nservos', 5);

px = RobotArm(L, arb, 'name', 'PhantomX', 'manufacturer', 'Trossen Robotics');
qz = [0 0 0 0];
px.tool = trotz(-pi/2) * trotx(pi/2);
