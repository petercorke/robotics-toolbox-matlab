%MDL_BAXTER Kinematic model of Baxter dual-arm robot
%
% MDL_BAXTER is a script that creates the workspace variables left and
% right which describes the kinematic characteristics of the two 7-joint
% arms of a Rethink Robotics Baxter robot using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qd         lower arm horizontal as per data sheet
%
% Notes::
% - SI units of metres are used.
%
% References::
% "Kinematics Modeling and Experimental Verification of Baxter Robot"
% Z. Ju, C. Yang, H. Ma, Chinese Control Conf, 2015.
%
% See also mdl_nao, SerialLink.

% Copyright (C) 1993-2017, by Peter I. Corke
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


% MODEL: Baxter, Rethink Robotics, 7DOF, standard_DH


%     th  d  a  alpha

links = [
        Revolute('d', 0.27,        'a', 0.069, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2, 'offset', pi/2)
        Revolute('d', 0.102+0.262, 'a', 0.069, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2)
        Revolute('d', 0.103+0.271, 'a', 0.010, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2)
        Revolute('d', 0.28,        'a', 0, 'alpha', 0)
];

left =  SerialLink(links, 'name', 'Baxter LEFT', 'manufacturer', 'Rethink Robotics');
right = SerialLink(links, 'name', 'Baxter RIGHT', 'manufacturer', 'Rethink Robotics');

left.base = transl(0.064614, 0.25858, 0.119)*rpy2tr(0, 0, pi/4, 'xyz');
right.base = transl(0.063534, -0.25966, 0.119)*rpy2tr(0, 0, -pi/4, 'xyz');

% define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0 0];
qn = [0 pi/4 pi/2 0 pi/4  0 0];
