%MDL_FANUC10L  Create kinematic model of Fanuc AM120iB/10L robot 
%
% MDL_FANUC10L is a script that creates the workspace variable R which
% describes the kinematic characteristics of a Fanuc AM120iB/10L robot
% using standard DH conventions.
%
% Also defines the workspace vector:
%   q0   mastering position.
%
% Notes::
% - SI units of metres are used.
%
% Author::
%  Wynand Swart,
%  Mega Robots CC, P/O Box 8412, Pretoria, 0001, South Africa
%  wynand.swart@gmail.com
%
% See also SerialLink, mdl_irb140, mdl_m16, mdl_motomanHP6, mdl_puma560.

% MODEL: Fanuc, AM120iB/10L, 6DOF, standard_DH

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

%Cell: 073-1555-430
%30 Sep 2007
%Fanuc AM120iB/10L robot

%            theta    d      a    alpha
clear L
L(1) = Link([0        0     0.15  -pi/2   0  ]);
L(2) = Link([0        0     0.77   pi     0  ]);
L(3) = Link([0        0     0.1   -pi/2   0  ]);
L(4) = Link([0       -0.96  0      pi/2   0  ]);
L(5) = Link([0        0     0     -pi/2   0  ]);
L(6) = Link([0       -0.1   0      0      0  ]);
%##########################################################
%Pose 0; At MASTERING position;
%##########################################################
q0 =[0   -pi/2   0   0   0   0];
R=SerialLink(L, 'name', 'Fanuc AM120iB/10L');
%##########################################################
