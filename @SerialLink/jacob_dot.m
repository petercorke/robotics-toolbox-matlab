%SerialLink.jacob_dot Derivative of Jacobian
%
% JDQ = R.jacob_dot(Q, QD) is the product (6x1) of the derivative of the
% Jacobian (in the world frame) and the joint rates.
%
% Notes::
% - Useful for operational space control XDD = J(Q)QDD + JDOT(Q)QD
% - Written as per the reference and not very efficient.
%
% References::
% - Fundamentals of Robotics Mechanical Systems (2nd ed)
%   J. Angleles, Springer 2003.
%
% See also SerialLink.jacob0, diff2tr, tr2diff.


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

function Jdot = jacob_dot(robot, q, qd)

	n = robot.n;
    links = robot.links;

    % Using the notation of Angeles:
    %   [Q,a] ~ [R,t] the per link transformation
    %   P ~ R   the cumulative rotation t2r(Tj) in world frame
    %   e       the last column of P, the local frame z axis in world coordinates
    %   w       angular velocity in base frame
    %   ed      deriv of e
    %   r       is distance from final frame
    %   rd      deriv of r
    %   ud      ??

    for i=1:n
        T = links(i).A(q(i));
        Q{i} = t2r(T);
        a{i} = transl(T);
    end

    P{1} = Q{1};
    e{1} = [0 0 1]';
    for i=2:n
        P{i} = P{i-1}*Q{i};
        e{i} = P{i}(:,3);
    end

    % step 1
    w{1} = qd(1)*e{1};
    for i=1:(n-1)
        w{i+1} = qd(i+1)*[0 0 1]' + Q{i}'*w{i};
    end

    % step 2
    ed{1} = [0 0 0]';
    for i=2:n
        ed{i} = cross(w{i}, e{i});
    end

    % step 3
    rd{n} = cross( w{n}, a{n});
    for i=(n-1):-1:1
        rd{i} = cross(w{i}, a{i}) + Q{i}*rd{i+1};
    end

    r{n} = a{n};
    for i=(n-1):-1:1
        r{i} = a{i} + Q{i}*r{i+1};
    end

    ud{1} = cross(e{1}, rd{1});
    for i=2:n
        ud{i} = cross(ed{i}, r{i}) + cross(e{i}, rd{i});
    end

    % step 4
    %  swap ud and ed
    v{n} = qd(n)*[ud{n}; ed{n}];
    for i=(n-1):-1:1
        Ui = blkdiag(Q{i}, Q{i});
        v{i} = qd(i)*[ud{i}; ed{i}] + Ui*v{i+1};
    end

    Jdot = v{1};
