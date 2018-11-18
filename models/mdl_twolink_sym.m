%MDL_TWOLINK_SYM Create symbolic model of a simple 2-link mechanism
%
% MDL_TWOLINK_SYM is a script that creates the workspace variable twolink
% which describes in symbolic form the kinematic and dynamic
% characteristics of a simple planar 2-link mechanism moving in the
% xz-plane, it experiences gravity loading.  The symbolic parameters are:
%  - link lengths: a1, a2
%  - link masses: m1, m2
%  - link CoMs in the link frame x-direction: c1, c2
%  - gravitational acceleration: g
%  - joint angles: q1, q2
%  - joint angle velocities: qd1, qd2
%  - joint angle accelerations: qdd1, qdd2
%
% Notes::
% - It is a planar mechanism operating in the vertical plane and is 
%   therefore affected by gravity (unlike mdl_planar2 in the horizontal
%   plane).
% - Gear ratio is 1 and motor inertia is 0.
% - Link inertias Iyy1, Iyy2 are 0.
% - Viscous and Coulomb friction is 0. 
%
% References::
%  - Based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
%
% See also mdl_puma560, mdl_stanford, SerialLink.

% MODEL: generic, planar, dynamics, 2DOF, symbolic, standard_DH


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


syms a1 a2 g real
syms c1 c2 m1 m2 real

%syms Iyy1 Iyy2 b1 b2 real
Iyy1 = 0
Iyy2 = 0
b1 = 0
b2 = 0

twolink = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'm', m1, 'r', [c1 0 0], 'I', [0 Iyy1 0], 'B', b1, 'G', 1, 'Jm', 0, 'standard')
    Revolute('d', 0, 'a', a2, 'alpha', 0, 'm', m2, 'r', [c2 0 0], 'I', [0 Iyy2 0], 'B', b2, 'G', 1, 'Jm', 0, 'standard')
    ], ...
    'name', 'two link', ...
    'comment', 'from Spong, Hutchinson, Vidyasagar');
twolink = twolink.sym();
twolink.gravity = [0; 0; g];
twolink.base = trotx(sym('pi')/2);

syms q1 q2 q1d q2d q1dd q2dd real
