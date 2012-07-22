%SerialLink.accel Manipulator forward dynamics
%
% QDD = R.accel(Q, QD, TORQUE) is a vector (Nx1) of joint accelerations that result 
% from applying the actuator force/torque to the manipulator robot in state Q and QD.
% If Q, QD, TORQUE are matrices (KxN) then QDD is a matrix (KxN) where each row 
% is the acceleration corresponding to the equivalent rows of Q, QD, TORQUE.
%
% QDD = R.accel(X) as above but X=[Q,QD,TORQUE].
%
% Note::
% - Uses the method 1 of Walker and Orin to compute the forward dynamics.
% - This form is useful for simulation of manipulator dynamics, in
%   conjunction with a numerical integration function.
%
% References::
% - Efficient dynamic computer simulation of robotic mechanisms,
%   M. W. Walker and D. E. Orin,
%   ASME Journa of Dynamic Systems, Measurement and Control, vol. 104, no. 3, pp. 205-211, 1982.
%
% See also SerialLink.rne, SerialLink, ode45.

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


function qdd = accel(robot, Q, qd, torque)

	n = robot.n;

    if nargin == 2
        if length(Q) ~= (3*robot.n)
            error('RTB:accel:badarg', 'Input vector X is length %d, should be %d (q, qd, tau)', length(Q), 3*robot.n);
        end
        % accel(X)
        Q = Q(:)';   % make it a row vector
	    q = Q(1:n);
		qd = Q(n+1:2*n);
		torque = Q(2*n+1:3*n);
    elseif nargin == 4
        % accel(Q, qd, torque)
        
        if numrows(Q) > 1
            % handle trajectory by recursion
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
        else
            q = Q';
            if length(q) == n
                q = q(:)';
                qd = qd(:)';
            end
            if numcols(Q) ~= n
                error('q must have %d columns', n);
            end
            if numcols(qd) ~= robot.n
                error('qd must have %d columns', n);
            end
            if numcols(torque) ~= robot.n
                error('torque must have %d columns', n);
            end
        end
    else
        error('RTB:accel:badargs', 'insufficient arguments');
    end


	% compute current manipulator inertia
	%   torques resulting from unit acceleration of each joint with
	%   no gravity.
	M = rne(robot, ones(n,1)*q, zeros(n,n), eye(n), [0;0;0]);

	% compute gravity and coriolis torque
	%    torques resulting from zero acceleration at given velocity &
	%    with gravity acting.
	tau = rne(robot, q, qd, zeros(1,n));	

	qdd = M \ (torque - tau)';