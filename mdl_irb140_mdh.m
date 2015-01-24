%MDL_IR140 Create model of the ABB IRB 140 manipulator
%
%      mdl_irb140_mod
%
% Script creates the workspace variable irb which describes the 
% kinematic characteristics of an ABB IRB 140 manipulator using 
% modified DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Reference::
% - ABB IRB 140 data sheet
% - "THE MODELING OF A SIX DEGREE-OF-FREEDOM INDUSTRIAL ROBOT FOR 
%   THE PURPOSE OF EFFICIENT PATH PLANNING"
%   Master of Science Thesis, Penn State U, May 2009
%   Tyler Carter
%
% See also SerialLink, mdl_irb140, mdl_puma560, mdl_stanford, mdl_twolink.
%
% Notes::
% - SI units of metres are used.
% - The tool frame is in the centre of the tool flange.
% - Zero angle configuration has the upper arm vertical and lower arm
%   horizontal.

% MODEL: ABB, IRB140, 6DOF, modified_DH

% Reference::



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

% joint angle limits from 
% A combined optimization method for solving the inverse kinematics problem...
% Wang & Chen
% IEEE Trans. RA 7(4) 1991 pp 489-
L(1) = Revolute('d', 0.352, 'a', 0, 'alpha', 0, 'offset', 0, 'modified');
L(2) = Revolute('d', 0, 'a', 0.070, 'alpha', pi/2, 'offset', 0, 'modified');
L(3) = Revolute('d', 0, 'a', 0.360, 'alpha', 0, 'offset', 0, 'modified');
L(4) = Revolute('d', 0.380, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'modified');
L(6) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, 'offset', 0, 'modified');

L(1).m = 34655.36e-3;
L(1).r = [27.87 43.12 -89.03]*1e-3;
L(1).I = [
    512052539.74 1361335.88 51305020.72
    1361335.88 464074688.59 70335556.04
    51305020.72 70335556.04 462745526.12]*1e-9;

L(2).m = 15994.59e-3;
L(2).r = [ 198.29 9.73 92.43]*1e03;
L(2).I = [
    94817914.40 -3859712.77 37932017.01
    -3859712.77 328604163.24 -1088970.86
    37932017.01 -1088970.86 277463004.88]*1e-9;

L(3).m = 20862.05e-3;
L(3).r = [ -4.56 -79.96 -5.86];
L(3).I = [
    500060915.95 -1863252.17 934875.78
    -1863252.17 75152670.69 -15204130.09
    934875.78 -15204130.09 515424754.34]*1e-9;

irb = SerialLink(L, 'name', 'IRB 140', ...
    'manufacturer', 'ABB', 'comment', 'modified DH');

%
% some useful poses
%
qz = [0 0 0 0 0 0]; % zero angles, L shaped pose

clear L
