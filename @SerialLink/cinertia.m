%SerialLink.cinertia Cartesian inertia matrix
%
% M = R.cinertia(Q) is the NxN Cartesian (operational space) inertia matrix which relates 
% Cartesian force/torque to Cartesian acceleration at the joint configuration Q, and N 
% is the number of robot joints.
%
% See also SerialLink.inertia, SerialLink.rne.

% MOD HISTORY
% 	4/99 add object support
% $Log: not supported by cvs2svn $
% $Revision: 1.2 $



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
%
% http://www.petercorke.com

function Mx = cinertia(robot, q)
	J = jacob0(robot, q);
	Ji = inv(J);                %#ok<*MINV>
	M = inertia(robot, q);
	Mx = Ji' * M * Ji;
