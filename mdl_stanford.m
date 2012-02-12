%MDL_STANFORD Create model of Stanford arm
%
%      mdl_stanford
%
% Script creates the workspace variable stanf which describes the 
% kinematic and dynamic characteristics of the Stanford (Scheinman) arm.
%
% Also defines the vectors:
%   qz   zero joint angle configuration.
%
% Note::
% - Gear ratios not currently known, though reflected armature inertia 
%   is known, so gear ratios are set to 1.
%
% References::
% - Kinematic data from "Modelling, Trajectory calculation and Servoing of 
%   a computer controlled arm".  Stanford AIM-177.  Figure 2.3
% - Dynamic data from "Robot manipulators: mathematics, programming and control"
%   Paul 1981, Tables 6.4, 6.6
% 
% See also SerialLink, mdl_puma560, mdl_puma560akb, mdl_twolink.


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

clear L
%             th    d       a    alpha
L(1) = Link([ 0     0.412   0   -pi/2     0]);
L(2) = Link([ 0     0.154   0    pi/2     0]);
L(3) = Link([ -pi/2 0       0    0        1]);
L(4) = Link([ 0     0       0   -pi/2     0]);
L(5) = Link([ 0     0       0    pi/2     0]);
L(6) = Link([ 0     0.263   0    0        0]);

L(1).m = 9.29;
L(2).m = 5.01;
L(3).m = 4.25;
L(4).m = 1.08;
L(5).m = 0.630;
L(6).m = 0.51;

L(1).Jm = 0.953;
L(2).Jm = 2.193;
L(3).Jm = 0.782;
L(4).Jm = 0.106;
L(5).Jm = 0.097;
L(6).Jm = 0.020;

L(1).G = 1;
L(2).G = 1;
L(3).G = 1;
L(4).G = 1;
L(5).G = 1;
L(6).G = 1;

L(1).I = [0.276   0.255   0.071   0   0   0];
L(2).I = [0.108   0.018   0.100   0   0   0];
L(3).I = [2.51    2.51    0.006   0   0   0 ];
L(4).I = [0.002   0.001   0.001   0   0   0 ];
L(5).I = [0.003   0.0004  0       0   0   0];
L(6).I = [0.013   0.013   0.0003  0   0   0];

L(1).r = [0    0.0175 -0.1105];
L(2).r = [0   -1.054  0];
L(3).r = [0    0      -6.447];
L(4).r = [0    0.092  -0.054];
L(5).r = [0    0.566   0.003];
L(6).r = [0    0       1.554];

qz = [0 0 0 0 0 0];

stanf = SerialLink(L, 'name', 'Stanford arm');
stanf.plotopt = {'workspace', [-2 2 -2 2 -2 2]};
