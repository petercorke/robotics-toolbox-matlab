%MDL_STANFORD_MDH Create model of Stanford arm using MDH conventions
%
%      mdl_stanford_mdh
%
% Script creates the workspace variable stanf which describes the 
% kinematic and dynamic characteristics of the Stanford (Scheinman) arm
% using modified Denavit-Hartenberg parameters.
%
% Also defines the vectors:
%   qz   zero joint angle configuration.
%
% Notes::
% - SI units are used.
%
% References::
% - Kinematic data from "Modelling, Trajectory calculation and Servoing of 
%   a computer controlled arm".  Stanford AIM-177.  Figure 2.3
% - Dynamic data from "Robot manipulators: mathematics, programming and control"
%   Paul 1981, Tables 6.5, 6.6
% 
% See also SerialLink, mdl_puma560, mdl_puma560akb.


% MODEL: Stanford, Stanford arm, prismatic, 6DOF, modified_DH


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

qz = [0 0 0 0 0 0];

stanf = SerialLink([
            RevoluteMDH('d', 0.412)
            RevoluteMDH('d', 0.154, 'alpha', -pi/2)
            PrismaticMDH('alpha', pi/2, 'qlim', [0.2032 0.9144])
            RevoluteMDH()
            RevoluteMDH('alpha', -pi/2)
            RevoluteMDH('d', 0.263, 'alpha', pi/2)
        ], ...
    'name', 'Stanford arm MDH', ...
    'plotopt', {'workspace', [-2 2 -2 2 -2 2]} ...
    );
