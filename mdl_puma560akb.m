%MDL_PUMA560AKB Create model of Puma 560 manipulator
%
% MDL_PUMA560AKB is a script that creates the workspace variable p560m
% which describes the kinematic and dynamic characterstics of a Unimation
% Puma 560 manipulator modified DH conventions.
%
% Also defines the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%
% Notes::
% - SI units are used.
%
% References::
% -  "The Explicit Dynamic Model and Inertial Parameters of the Puma 560 Arm"
%    Armstrong, Khatib and Burdick
%    1986
%
% See also SerialLink, mdl_puma560, mdl_stanford_mdh.

% MODEL: Unimation, Puma560, dynamics, 6DOF, modified_DH


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
%            theta    d        a    alpha
L(1) = Link([  0      0        0       0       0], 'modified');
L(2) = Link([  0      0.2435   0      -pi/2    0], 'modified');
L(3) = Link([  0     -0.0934   0.4318  0       0], 'modified');
L(4) = Link([  0      0.4331  -0.0203  pi/2    0], 'modified');
L(5) = Link([  0      0        0      -pi/2    0], 'modified');
L(6) = Link([  0      0        0       pi/2    0], 'modified');


L(1).m = 0;
L(2).m = 17.4;
L(3).m = 4.8;
L(4).m = 0.82;
L(5).m = 0.34;
L(6).m = .09;

%         rx      ry      rz
L(1).r = [0   0   0 ];
L(2).r = [0.068   0.006   -0.016];
L(3).r = [0   -0.070  0.014 ];
L(4).r = [0   0   -0.019];
L(5).r = [0   0   0 ];
L(6).r = [0   0   .032  ];

%        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
L(1).I = [0   0   0.35    0   0   0];
L(2).I = [.13   .524    .539    0     0   0];
L(3).I = [.066    .0125   .066    0   0   0];
L(4).I = [1.8e-3  1.8e-3  1.3e-3  0   0   0];
L(5).I = [.3e-3   .3e-3   .4e-3   0   0   0];
L(6).I = [.15e-3  .15e-3  .04e-3  0   0   0];

L(1).Jm =  291e-6;
L(2).Jm =  409e-6;
L(3).Jm =  299e-6;
L(4).Jm =  35e-6;
L(5).Jm =  35e-6;
L(6).Jm =  35e-6;

L(1).G =  -62.6111;
L(2).G =  107.815;
L(3).G =  -53.7063;
L(4).G =  76.0364;
L(5).G =  71.923;
L(6).G =  76.686;

% viscous friction (motor referenced)
% unknown

% Coulomb friction (motor referenced)
% unknown

%
% some useful poses
%
qz = [0 0 0 0 0 0]; % zero angles, L shaped pose
qr = [0 -pi/2 pi/2 0 0 0]; % ready pose, arm up
qstretch = [0 0 pi/2 0 0 0]; % horizontal along x-axis

p560m = SerialLink(L, 'name', 'Puma560-AKB', 'manufacturer', 'Unimation', 'comment', 'AK&B');
clear L
