
%SERIALLINK.RNE_MDH Compute inverse dynamics via recursive Newton-Euler formulation
%
% Recursive Newton-Euler for modified Denavit-Hartenberg notation.  Is invoked by
% R.RNE().
%
% See also SERIALLINK.RNE.





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

function tau = rne_mdh(robot, a1, a2, a3, a4, a5)

	z0 = [0;0;1];
	grav = robot.gravity;	% default gravity from the object
	fext = zeros(6, 1);

	% Set debug to:
	%	0 no messages
	%	1 display results of forward and backward recursions
	%	2 display print R and p*
	debug = 0;

	n = robot.n;
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
	
    if robot.issym || any([isa(Q,'sym'), isa(Qd,'sym'), isa(Qdd,'sym')])
        tau = zeros(np,n, 'sym');
    else
        tau = zeros(np,n);
    end

	for p=1:np
		q = Q(p,:).';
		qd = Qd(p,:).';
		qdd = Qdd(p,:).';
	
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
            switch link.type
                case 'R'
                    D = link.d;
                case 'P'
                    D = q(j);
            end
			alpha = link.alpha;
			pm = [link.a; -D*sin(alpha); D*cos(alpha)];	% (i-1) P i
			if j == 1
				pm = t2r(robot.base) * pm;
				Tj = robot.base * Tj;
			end
			Pm(:,j) = pm;
			Rm{j} = t2r(Tj);
			if debug>1
				Rm{j}
				Pm(:,j).'
			end
		end

	%
	%  the forward recursion
	%
		for j=1:n
			link = robot.links(j);

			R = Rm{j}.';	% transpose!!
			P = Pm(:,j);
            Pc = link.r;
            
            %
            % trailing underscore means new value
            %
            switch link.type
                case 'R'
                    % revolute axis
                    w_ = R*w + z0*qd(j);
                    wd_ = R*wd + cross(R*w,z0*qd(j)) + z0*qdd(j);
                    %v = cross(w,P) + R*v;
                    vd_ = R * (cross(wd,P) + ...
                        cross(w, cross(w,P)) + vd);
                    
                case 'P'
                    % prismatic axis
                    w_ = R*w;
                    wd_ = R*wd;
                    %v = R*(z0*qd(j) + v) + cross(w,P);
                    vd_ = R*(cross(wd,P) + ...
                        cross(w, cross(w,P)) + vd ...
                        ) + 2*cross(R*w,z0*qd(j)) + z0*qdd(j);
            end
			% update variables
			w = w_;
			wd = wd_;
			vd = vd_;

			vdC = cross(wd,Pc).' + ...
				cross(w,cross(w,Pc)).' + vd;
			F = link.m*vdC;
			N = link.I*wd + cross(w,link.I*w);
			Fm = [Fm F];
			Nm = [Nm N];
			if debug
				fprintf('w: '); fprintf('%.3f ', w)
				fprintf('\nwd: '); fprintf('%.3f ', wd)
				fprintf('\nvd: '); fprintf('%.3f ', vd)
				fprintf('\nvdbar: '); fprintf('%.3f ', vdC)
				fprintf('\n');
			end
		end

	%
	%  the backward recursion
	%

		fext = fext(:);
		f = fext(1:3);		% force/moments on end of arm
		nn = fext(4:6);

		for j=n:-1:1
			
			%
			% order of these statements is important, since both
			% nn and f are functions of previous f.
			%
			link = robot.links(j);
			
			if j == n
				R = eye(3,3);
				P = [0;0;0];
			else
				R = Rm{j+1};
				P = Pm(:,j+1);		% i/P/(i+1)
			end
			Pc = link.r;
			
			f_ = R*f + Fm(:,j);
			nn_ = Nm(:,j) + R*nn + cross(Pc,Fm(:,j)).' + ...
				cross(P,R*f);
			
			f = f_;
			nn = nn_;

			if debug
				fprintf('f: '); fprintf('%.3f ', f)
				fprintf('\nn: '); fprintf('%.3f ', nn)
				fprintf('\n');
			end
            switch link.type
                case 'R'
                    % revolute
                    tau(p,j) = nn.'*z0 + ...
                        link.G^2 * link.Jm*qdd(j) - ...
                        friction(link, qd(j));
                case 'P'
                    % prismatic
                    tau(p,j) = f.'*z0 + ...
                        link.G^2 * link.Jm*qdd(j) - ...
                        friction(link, qd(j));
            end
		end
	end
