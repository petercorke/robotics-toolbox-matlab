%SerialLink.accel Manipulator forward dynamics
%
% QDD = R.accel(Q, QD, TORQUE) is a vector (Nx1) of joint accelerations that result 
% from applying the actuator force/torque to the manipulator robot in state Q and QD.
% If Q, QD, TORQUE are matrices with M rows, then QDD is a matrix with M rows
% of acceleration corresponding to the equivalent rows of Q, QD, TORQUE.
%
% QDD = R.ACCEL(X) as above but X=[Q,QD,TORQUE].
%
% Note::
% - Uses the method 1 of Walker and Orin to compute the forward dynamics.
% - This form is useful for simulation of manipulator dynamics, in
%   conjunction with a numerical integration function.
%
% See also SerialLink.rne, SerialLink, ode45.

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


function qdd = accel(robot, Q, qd, torque)

    if numcols(Q) ~= robot.n
        error('q must have %d columns', robot.n);
    end
    if numcols(qd) ~= robot.n
        error('qd must have %d columns', robot.n);
    end
    if numcols(torque) ~= robot.n
        error('torque must have %d columns', robot.n);
    end

    if numrows(Q) > 1
        if numrows(Q) ~= numrows(qd)
            error('for trajectory q and qd must have same number of rows');
        end
        if numrows(Q) ~= numrows(torque)
            error('for trajectory q and torque must have same number of rows');
        end
        qdd = [];
        for i=1:numrows(Q)
            qdd = cat(1, qdd, robot.accel(Q(i,:), qd(i,:), torque(i,:))');
        end
        return
    end

	n = robot.n;

	if nargin == 2
        % accel(X)
	    q = Q(1:n);
		qd = Q(n+1:2*n);
		torque = Q(2*n+1:3*n);
	else
        % accel(Q, qd, torque)
		q = Q;
		if length(q) == robot.n,
			q = q(:);
			qd = qd(:);
		end
	end

	% compute current manipulator inertia
	%   torques resulting from unit acceleration of each joint with
	%   no gravity.
	M = rne(robot, ones(n,1)*q', zeros(n,n), eye(n), [0;0;0]);

	% compute gravity and coriolis torque
	%    torques resulting from zero acceleration at given velocity &
	%    with gravity acting.
	tau = rne(robot, q', qd', zeros(1,n));	

	qdd = inv(M) * (torque(:) - tau');

