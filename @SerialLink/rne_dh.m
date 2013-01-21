
%SERIALLINK.RNE_DH Compute inverse dynamics via recursive Newton-Euler formulation
%
% Recursive Newton-Euler for standard Denavit-Hartenberg notation.  Is invoked by
% R.RNE().
%
% See also SERIALLINK.RNE.

%
% verified against MAPLE code, which is verified by examples
%



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
function [tau,wbase] = rne_dh(robot, a1, a2, a3, a4, a5)

    z0 = [0;0;1];
    grav = robot.gravity;   % default gravity from the object
    fext = zeros(6, 1);
    n = robot.n;

    % check that robot object has dynamic parameters for each link
    for j=1:n
        link = robot.links(j);
        if isempty(link.r) || isempty(link.I) || isempty(link.m)
            error('dynamic parameters (m, r, I) not set in link %d', j);
        end
    end

    % Set debug to:
    %   0 no messages
    %   1 display results of forward and backward recursions
    %   2 display print R and p*
    debug = 0;

    if numcols(a1) == 3*n
        Q = a1(:,1:n);
        Qd = a1(:,n+1:2*n);
        Qdd = a1(:,2*n+1:3*n);
        np = numrows(Q);
        if nargin >= 3, 
            grav = a2(:);
        end
        if nargin == 4
            fext = a3;
        end
    else
        np = numrows(a1);
        Q = a1;
        Qd = a2;
        Qdd = a3;
        if numcols(a1) ~= n || numcols(Qd) ~= n || numcols(Qdd) ~= n || ...
            numrows(Qd) ~= np || numrows(Qdd) ~= np
            error('bad data');
        end
        if nargin >= 5, 
            grav = a4(:);
        end
        if nargin == 6
            fext = a5;
        end
    end
    
    if isa(Q, 'sym')
        tau(np, n) = sym();
    else
        tau = zeros(np,n);
    end

    for p=1:np
        q = Q(p,:).';
        qd = Qd(p,:).';
        qdd = Qdd(p,:)';
    
        Fm = [];
        Nm = [];
        pstarm = [];
        Rm = [];
        w = zeros(3,1);
        wd = zeros(3,1);
        vd = grav(:);

    %
    % init some variables, compute the link rotation matrices
    %
        for j=1:n
            link = robot.links(j);
            Tj = link.A(q(j));
            if link.RP == 'R'
                d = link.d;
            else
                d = q(j);
            end
            alpha = link.alpha;
            pstar = [link.a; d*sin(alpha); d*cos(alpha)];
            if j == 1
                pstar = t2r(robot.base) * pstar;
                Tj = robot.base * Tj;
            end
            pstarm(:,j) = pstar;
            Rm{j} = t2r(Tj);
            if debug>1
                Rm{j}
                Pstarm(:,j).'
            end
        end

    %
    %  the forward recursion
    %
        for j=1:n
            link = robot.links(j);

            Rt = Rm{j}.';    % transpose!!
            pstar = pstarm(:,j);
            r = link.r;

            %
            % statement order is important here
            %
            if link.RP == 'R'
                % revolute axis
                wd = Rt*(wd + z0*qdd(j) + ...
                    cross(w,z0*qd(j)));
                w = Rt*(w + z0*qd(j));
                %v = cross(w,pstar) + Rt*v;
                vd = cross(wd,pstar) + ...
                    cross(w, cross(w,pstar)) +Rt*vd;

            else
                % prismatic axis
                w = Rt*w;
                wd = Rt*wd;
                vd = Rt*(z0*qdd(j)+vd) + ...
                    cross(wd,pstar) + ...
                    2*cross(w,Rt*z0*qd(j)) +...
                    cross(w, cross(w,pstar));
            end

            %whos
            vhat = cross(wd,r.') + ...
                cross(w,cross(w,r.')) + vd;
            F = link.m*vhat;
            N = link.I*wd + cross(w,link.I*w);
            Fm = [Fm F];
            Nm = [Nm N];

            if debug
                fprintf('w: '); fprintf('%.3f ', w)
                fprintf('\nwd: '); fprintf('%.3f ', wd)
                fprintf('\nvd: '); fprintf('%.3f ', vd)
                fprintf('\nvdbar: '); fprintf('%.3f ', vhat)
                fprintf('\n');
            end
        end

    %
    %  the backward recursion
    %

        fext = fext(:);
        f = fext(1:3);      % force/moments on end of arm
        nn = fext(4:6);

        for j=n:-1:1
            link = robot.links(j);
            pstar = pstarm(:,j);
            
            %
            % order of these statements is important, since both
            % nn and f are functions of previous f.
            %
            if j == n
                R = eye(3,3);
            else
                R = Rm{j+1};
            end
            r = link.r;
            nn = R*(nn + cross(R.'*pstar,f)) + ...
                cross(pstar+r.',Fm(:,j)) + ...
                Nm(:,j);
            f = R*f + Fm(:,j);
            if debug
                fprintf('f: '); fprintf('%.3f ', f)
                fprintf('\nn: '); fprintf('%.3f ', nn)
                fprintf('\n');
            end

            R = Rm{j};
            if link.RP == 'R'
                % revolute
                %tau(p,j) = nn.'*(R.'*z0) + ...
                t = nn.'*(R.'*z0) + ...
                    link.G^2 * link.Jm*qdd(j) - ...
                     link.friction(qd(j));
                tau(p,j) = t;
            else
                % prismatic
                t = f.'*(R.'*z0) + ...
                    link.G^2 * link.Jm*qdd(j) - ...
                    link.friction(qd(j));
                tau(p,j) = t;
            end
        end
        % this last bit needs work/testing
        R = Rm{1};
        nn = R*(nn);
        f = R*f;
        wbase = [f; nn];
    end
    
    if isa(tau, 'sym')
        tau = simplify(tau);
    end
