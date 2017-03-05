%SerialLink.itorque Inertia torque
%
% TAUI = R.itorque(Q, QDD) is the inertia force/torque vector (1xN) at the
% specified joint configuration Q (1xN) and acceleration QDD (1xN), and N
% is the number of robot joints. TAUI = INERTIA(Q)*QDD.
%
% If Q and QDD are matrices (KxN), each row is interpretted as a joint state 
% vector, and the result is a matrix (KxN) where each row is the corresponding
% joint torques.
% 
% Note::
% - If the robot model contains non-zero motor inertia then this will 
%   included in the result.
%
% See also SerialLink.inertia, SerialLink.rne.





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

function it = itorque(robot, q, qdd)
	it = rne(robot, q, zeros(size(q)), qdd, 'gravity', [0;0;0]);
