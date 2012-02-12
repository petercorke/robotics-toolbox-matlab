%SerialLink.coriolis Coriolis matrix
%
% C = R.CORIOLIS(Q, QD) is the NxN Coriolis/centripetal matrix for
% the robot in configuration Q and velocity QD, where N is the number of
% joints.  The product C*QD is the vector of joint force/torque due to velocity
% coupling.  The diagonal elements are due to centripetal effects and the 
% off-diagonal elements are due to Coriolis effects.  This matrix is also 
% known as the velocity coupling matrix, since gives the disturbance forces
% on all joints due to velocity of any joint.
%
% If Q and QD are matrices (DxN), each row is interpretted as a joint state 
% vector, and the result (NxNxD) is a 3d-matrix where each plane corresponds
% to a row of Q and QD.
%
% Notes::
% - joint friction is also a joint force proportional to velocity but it is
%   eliminated in the computation of this value.
% - computationally slow, involves N^2/2 invocations of RNE.
%
% See also SerialLink.rne.

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

function C = coriolis(robot, q, qd)

    if numcols(q) ~= robot.n
        error('q must have %d columns', robot.n);
    end
    if numcols(qd) ~= robot.n
        error('qd must have %d columns', robot.n);
    end

    % we need to create a clone robot with no friciton, since friction
    % is also proportional to joint velocity
    robot2 = nofriction(robot,'all');

    if numrows(q) > 1
        if numrows(q) ~= numrows(qd)
            error('for trajectory q and qd must have same number of rows');
        end
        C = [];
        for i=1:numrows(q)
            C = cat(3, C, robot2.coriolis(q(i,:), qd(i,:)));
        end
        return
    end

    N = robot2.n;
    C = zeros(N,N);
    Csq = zeros(N,N);

    % find the torques that depend on a single finite joint speed,
    % these are due to the squared (centripetal) terms
    %
    %  set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
    for j=1:N
        QD = zeros(1,N);
        QD(j) = 1;
        tau = rne(robot2,q, QD, zeros(size(q)), [0 0 0]');
        Csq(:,j) = Csq(:,j) + tau';
    end

    % find the torques that depend on a pair of finite joint speeds,
    % these are due to the product (Coridolis) terms
    %  set QD = [1 1 0 ...] then resulting torque is due to 
    %    qd_1 qd_2 + qd_1^2 + qd_2^2
    for j=1:N
        for k=j+1:N
            % find a product term  qd_j * qd_k
            QD = zeros(1,N);
            QD(j) = 1;
            QD(k) = 1;
            tau = rne(robot2,q, QD, zeros(size(q)), [0 0 0]');
            C(:,k) = C(:,k) + (tau' - Csq(:,k) - Csq(:,j)) * qd(j);
        end
    end
    C = C + Csq * diag(qd);
