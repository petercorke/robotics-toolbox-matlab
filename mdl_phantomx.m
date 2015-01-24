%MDL_PHANTOMX Create model of PhantomX pincher manipulator
%
% MDL_PHANTOMX is a script that creates the workspace variable px which
% describes the kinematic characteristics of a PhantomX Pincher Robot, a 4
% joint hobby class  manipulator by Trossen Robotics.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Notes::
% - Uses standard DH conventions.
% - Tool centrepoint is middle of the fingertips.
% - All translational units in mm.
%
% Reference::
%
% - http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-basic-robot-arm.html

% MODEL: Trossen Robotics, PhantomX Pincher, 4DOF, standard_DH

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

px = SerialLink(L, 'name', 'PhantomX', 'manufacturer', 'Trossen Robotics');
qz = [0 0 0 0];
px.tool = trotz(-pi/2) * trotx(pi/2);
