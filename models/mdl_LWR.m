%MDL_LWR Create model of Kuka LWR manipulator
%
% MDL_LWR is a script that creates the workspace variable KR5 which
% describes the kinematic characteristics of a Kuka KR5 manipulator using
% standard DH conventions.
%
% Also define the workspace vectors:
%   qz        all zero angles
%
% Notes::
% - SI units of metres are used.
%
% Reference::
% - Identifying the Dynamic Model Used by the KUKA LWR: A Reverse Engineering Approach
%   Claudio Gaz Fabrizio Flacco Alessandro De Luca
%   ICRA 2014
%
% See also mdl_kr5, mdl_irb140, mdl_puma560, SerialLink.

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

% MODEL: Kuka, LWR, 7DOF, standard_DH

d1 =0.4; d2 = 0.39;

%All link lengths and offsets are measured in m
clear L
%            theta    d           a       alpha
links = [
	    Link([0        0           0       pi/2])
		Link([0        0           0      -pi])
		Link([0        d1          0      -pi/2])
		Link([0        0           0       pi/2])
		Link([0        d2          0       pi/2])
		Link([0        0           0       -pi/2])
		Link([0        0           0       0])
	];

LWR=SerialLink(links, 'name', 'Kuka LWR');

qz = [0 0 0 0 0 0 0];
