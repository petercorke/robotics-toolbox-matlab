%MDL_COBRA600 Create model of Adept Cobra 600 manipulator
%
% MDL_COBRA600 is a script that creates the workspace variable c600 which
% describes the kinematic characteristics of the 4-axis Adept Cobra 600
% SCARA manipulator using standard DH conventions. 
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Notes::
% - SI units are used.
%
% See also SerialRevolute, mdl_puma560akb, mdl_stanford.

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

% MODEL: Adept, Cobra600, 4DOF, standard_DH

% hardstop limits included
links = [
    Revolute('d', 0.387, 'a', 0.325, 'qlim', [-50 50]*pi/180);
    Revolute('a', 0.275, 'alpha', pi, 'qlim', [-88 88]*pi/180);
    Prismatic('qlim', [0 0.210]);
    Revolute()
    ];

c600 = SerialLink(links, 'name', 'Cobra600', 'manufacturer', 'Adept', ...
    'plotopt', {'workspace', [0 0.8 -0.6 0.6 0 0.4]} );

qz = [0 0 0 0];