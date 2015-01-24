%MDL_S4ABB2p8  Create kinematic model of ABB S4 2.8robot 
%
% MDL_S4ABB2p8 is a script creates the workspace variable R which describes
% the kinematic characteristics of an ABB S4 2.8 robot using standard DH
% conventions.
%
% Also defines the workspace vector:
%   q0   mastering position.
%
% Author::
%  Wynand Swart,
%  Mega Robots CC, P/O Box 8412, Pretoria, 0001, South Africa
%  wynand.swart@gmail.com
%
% See also SerialLink, mdl_fanuc10l, mdl_m16, mdl_motormanHP6, mdl_irb140, mdl_puma560.

% MODEL: ABB, S4_2.8, S4 2.8m reach version, 6DOF, standard_DH

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
%S4 ABB 2.8 robot

%            theta    d      a    alpha
clear L
L(1) = Link([ 0      0.9    0.188  -pi/2   0]);
L(2) = Link([ 0      0      0.95    0      0]);
L(3) = Link([ 0      0      0.225  -pi/2   0]);
L(4) = Link([ 0      1.705  0       pi/2   0]);
L(5) = Link([ 0      0      0      -pi/2   0]);
L(6) = Link([ 0      0.2    0      -pi/2   0]);
%##########################################################
%Pose 0; At SYNCHRONISATION position
%##########################################################
q0 = [0     -pi/2         0       0      0     -pi/2];
R=SerialLink(L, 'name', 'S4 ABB 2.8');
%##########################################################
