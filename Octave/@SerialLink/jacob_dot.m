%SerialLink.jacob_dot Hessian in end-effector frame
%
% JDQ = R.jacob_dot(Q, QD) is the product of the Hessian, derivative of the
% Jacobian, and the joint rates.
%
% Notes::
% - useful for operational space control
% - not yet tested/debugged.
%
% See also: SerialLink.jacob0, diff2tr, tr2diff.

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

function Jdot = jacob_dot(robot, q, qd)

	n = robot.n;
	L = robot.links;		% get the links

    Tj = robot.base;     % this is cumulative transform from base
    w = [0;0;0];
    v = [0;0;0];
    for j=1:n,
        link = robot.links(j);
        Aj = link.A(q(j));
        Tj = Tj * Aj;
        R{j} = t2r(Aj);
        T{j} = t2r(Tj);
        p(:,j) = transl(Tj);    % origin of link j

        if j>1,
            z(:,j) = T{j-1} * [0 0 1]'; % in world frame
            w = R{j}*( w + z(:,j) * qd(j));
            v = v + cross(R{j}*w, R{j-1}*p(:,j));
            %v = R{j-1}'*v + cross(w, p(:,j));
            %w = R{j-1}'* w + z(:,j) * qd(j);
        else
            z(:,j) = [0 0 1]';
            v = [0 0 0]';
            w = z(:,j) * qd(j);
        end

        vel(:,j) = v;       % velocity of origin of link j

        omega(:,j) = w;         % omega of link j in link j frame
    end
    omega
    z

    J = [];
    Jdot = [];
    for j=1:n,
        if j>1,
            t = p(:,n) - p(:,j-1);
            td = vel(:,n) - vel(:,j-1);
        else
            t = p(:,n);
            td = vel(:,n);
        end
        if L(j).RP == 'R',
            J_col = [cross(z(:,j), t); z(:,j)];
            Jdot_col = [cross(cross(omega(:,j), z(:,j)), t) + cross(z(:,j), td) ; cross(omega(:,j),  z(:,j))];
        else
            J_col = [z(:,j); 0;0;0];
            Jdot_col = zeros(6,1);
        end

        J = [J J_col];
        Jdot = [Jdot Jdot_col];

	end
