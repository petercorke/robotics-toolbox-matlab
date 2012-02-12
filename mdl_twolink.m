%MDL_TWOLINK Create model of a simple 2-link mechanism
%
%      mdl_twolink
%
% Script creates the workspace variable tl which describes the 
% kinematic and dynamic characteristics of a simple planar 2-link mechanism.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
% - Assume unit length links with all mass (unity) concentrated at the joints.
%
% References::
%  - Based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
%
% See also SerialLink, mdl_puma560, mdl_stanford.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

twolink_dh = [
% theta d a alpha a	sigma	m	rx	ry	rz	Ixx	Iyy	Izz	Ixy	Iyz	Ixz	Jm	G
  0     0         1     0         0     1       1       0       0       0       0       0       0       0       0        0      1
  0     0         1     0         0     1       1       0       0       0       0       0       0       0       0        0      1
];

a1 = 1;
a2 = 1;
%   theta d a alpha
L(1) = Link([ 0     0   a1  0], 'standard');
L(2) = Link([ 0     0   a2  0], 'standard');
L(1).m = 1;
L(1).r = [-0.5 0 0];
L(1).I = zeros(3,3);
L(1).G = 0;
L(1).Jm = 0;
L(1).B = 0;
L(2).m = 1;
L(2).r = [-0.5 0 0];
L(2).I = zeros(3,3);
L(2).G = 0;
L(2).Jm = 0;
L(2).B = 0;
twolink = SerialLink(L, 'name', 'two link', ...
    'comment', 'from Spong, Hutchinson, Vidyasagar');
qz = [0 0];
qn = [pi/6, -pi/6];
