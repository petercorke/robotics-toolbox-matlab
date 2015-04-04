%MDL_TWOLINK_SYM Create symbolic model of a simple 2-link mechanism
%
%      mdl_twolink_sym
%
% Script creates the workspace variable twolink which describes the 
% kinematic and dynamic characteristics of a simple planar 2-link mechanism in
% symbolic form.  The symbolic parameters are:
%  * link lengths: a1, a2
%  * link masses: m1, m2
%  * link CoMs in the link frame x-direction: c1, c2
%  * link inertias: Iyy1, Iyy2
%  * viscous friction: b1, b2
%  * gravitational acceleration: g
%  * joint angles: q1, q2
%  * joint angle velocities: qd1, qd2
%  * joint angle accelerations: qdd1, qdd2
%
% Gear ratio is 1 and motor inertia is 0.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
%
% References::
%  - Based on Fig 3-6 (p73) of Spong and Vidyasagar (1st edition).  
%
% See also SerialLink, mdl_puma560, mdl_stanford.

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


syms a1 a2 g
syms c1 c2 m1 m2 Iyy1 Iyy2 b1 b2

twolink = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'm', m1, 'r', [c1 0 0], 'I', [0 Iyy1 0], 'B', b1, 'G', 1, 'Jm', 0, 'standard')
    Revolute('d', 0, 'a', a2, 'alpha', 0, 'm', m2, 'r', [c2 0 0], 'I', [0 Iyy2 0], 'B', b2, 'G', 1, 'Jm', 0, 'standard')
    ], ...
    'name', 'two link', ...
    'comment', 'from Spong, Hutchinson, Vidyasagar');
twolink = twolink.sym();
twolink.gravity = [0; 0; g];
twolink.base = trotx(sym('pi')/2);
