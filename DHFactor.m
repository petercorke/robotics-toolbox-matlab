%DHFactor Simplify symbolic link transform expressions
%
% F = DHFactor(S) is an object that encodes the kinematic model of a robot
% provided by a string S that represents a chain of elementary transforms from 
% the robot's base to its tool tip.  The chain of elementary rotations and
% translations is symbolically factored into a sequence of link transforms 
% described by DH parameters.
%
% For example:
%    s = 'Rz(q1).Rx(q2).Ty(L1).Rx(q3).Tz(L2)';
% indicates a rotation of q1 about the z-axis, then rotation of q2 about the
% x-axis, translation of L1 about the y-axis, rotation of q3 about the x-axis
% and translation of L2 along the z-axis.
%
% Methods::
%
% base      the base transform as a Java string
% tool      the tool transform as a Java string
% command   a command string that will create a SerialLink() object 
%           representing the specified kinematics
% char      convert to string representation
% display   display in human readable form
%
% Example::
%
%    >> s = 'Rz(q1).Rx(q2).Ty(L1).Rx(q3).Tz(L2)';
%    >> dh = DHFactor(s);
%    >> dh
%    DH(q1+90, 0, 0, +90).DH(q2, L1, 0, 0).DH(q3-90, L2, 0, 0).Rz(+90).Rx(-90).Rz(-90)
%    >> r = eval( dh.command('myrobot') );
%
% Notes::
% - Variables starting with q are assumed to be joint coordinates.
% - Variables starting with L are length constants.
% - Length constants must be defined in the workspace before executing the 
%   last line above.
% - Implemented in Java.
% - Not all sequences can be converted to DH format, if conversion cannot be 
%   achieved an error is generated.
%
% Reference::
% - A simple and systematic approach to assigning Denavit-Hartenberg parameters,
%   P.Corke, IEEE Transaction on Robotics, vol. 23, pp. 590-594, June 2007.
% - Robotics, Vision & Control, Sec 7.5.2, 7.7.1,
%   Peter Corke, Springer 2011.
%
% See also SerialLink.


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
% See also SerialLink.
