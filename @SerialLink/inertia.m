%SerialLink.INERTIA Manipulator inertia matrix
%
% I = R.inertia(Q) is the symmetric joint inertia matrix (NxN) which relates 
% joint torque to joint acceleration for the robot at joint configuration Q.
%
% If Q is a matrix (KxN), each row is interpretted as a joint state 
% vector, and the result is a 3d-matrix (NxNxK) where each plane corresponds
% to the inertia for the corresponding row of Q.
%
% Notes::
% - The diagonal elements I(J,J) are the inertia seen by joint actuator J.
% - The off-diagonal elements I(J,K) are coupling inertias that relate 
%   acceleration on joint J to force/torque on joint K.
% - The diagonal terms include the motor inertia reflected through the gear
%   ratio.
%
% See also SerialLink.RNE, SerialLink.CINERTIA, SerialLink.ITORQUE.


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

function M = inertia(robot, q)
    if numcols(q) ~= robot.n
        error('q must have %d columns', robot.n);
    end

    if numrows(q) > 1
        M = [];
        for i=1:numrows(q)
            M = cat(3, M, robot.inertia(q(i,:)));
        end
        return
    end

	n = robot.n;

	if numel(q) == robot.n
		q = q(:).';
	end

	M = zeros(n,n,0);
	for Q = q.'
		m = rne(robot, ones(n,1)*Q.', zeros(n,n), eye(n), [0;0;0]);
		M = cat(3, M, m);
	end
