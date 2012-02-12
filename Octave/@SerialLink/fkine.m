%SerialLink.fkine  Forward kinematics
%
% T = R.fkine(Q) is the pose of the robot end-effector as a homogeneous
% transformation for the joint configuration Q.  For an N-axis manipulator 
% Q is an N-vector.
% 
% If Q is a matrix, the M rows are interpretted as the generalized 
% joint coordinates for a sequence of points along a trajectory.  Q(i,j) is
% the j'th joint parameter for the i'th trajectory point.  In this case
% it returns a 4x4xM matrix where the last subscript is the index
% along the path.
%
% Note::
% - The robot's base or tool transform, if present, are incorporated into the
%   result.
%
% See also SerialLink.ikine, SerialLink.ikine6s.

% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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

function t = fkine(robot, q)
	%
	% evaluate fkine for each point on a trajectory of 
	% theta_i or q_i data
	%

	n = robot.n;

	L = robot.links;
	if numel(q) == n
		t = robot.base;
		for i=1:n
			t = t * L(i).A(q(i));
		end
		t = t * robot.tool;
	else
		if numcols(q) ~= n
			error('q must have %d columns', n)
		end
		t = zeros(4,4,0);
		for qv=q',		% for each trajectory point
			tt = robot.base;
			for i=1:n
				tt = tt * L(i).A(qv(i));
			end
			t = cat(3, t, tt * robot.tool);
		end
	end

    %robot.T = t;
    %robot.notify('Moved');
