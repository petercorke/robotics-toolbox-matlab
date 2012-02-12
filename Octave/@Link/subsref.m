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
function v = subsref(l, s)
	switch s(1).type  
	case '.'
		% NOTE WELL:  the following code can't use getfield() since
		% getfield()  uses this, and Matlab will crash!!

		el = char(s(1).subs);
		switch el,
		case 'alpha',
			v = l.alpha;
		case 'a',
			v = l.a;
		case 'theta',
			v = l.theta;
		case 'd',
			v = l.d;
		case 'offset',
			v = l.offset;
		case 'sigma',
			v = l.sigma;
		case 'RP',
			if l.sigma == 0,
				v = 'R';
			else
				v = 'P';
			end
		case 'mdh',
			v = l.mdh;
		case 'G',
			v = l.G;
		case 'I',
			v = l.I;
		case 'r',
			v = l.r;
		case 'Jm',
			v = l.Jm;
		case 'B',
			v = l.B;
		case 'Tc',
			v = l.Tc;
		case 'qlim',
			v = l.qlim;
		case 'islimit',
			if s(2).type  ~= '()'
				error('expecting argument for islimit method');
			end
			q = s(2).subs{1};
			v = (q > l.qlim(2)) - (q < l.qlim(1));
		case 'm',
			v = l.m;
		case 'dh',
			v = [l.alpha l.A l.theta l.D l.sigma];
		case 'dyn',
			dyn(l);
		case 'A',
			if s(2).type  ~= '()'
				error('expecting argument for A method');
            else 
          
				args = s(2).subs;
				v = A(l,args{1});
            end
        
        case 'nofriction',
            q = s(2).subs;
            v = nofriction(l,q{:});
		otherwise, 
			disp('Unknown method ref')
		end
	case '()'
								
		if numel(s) == 1
			
			v = builtin('subsref', l, s);
		else
			z = s(1).subs;  % cell array
		
			k = z{1};
			l = l(k);
			v_subset = subsref(l, s(2:end));
			
			v = v_subset;  % put subset back;
		end
	otherwise 
		error('only .field supported')
	end %switch
	

endfunction

function T = A(L, q)
%Link.A Link transform matrix
%
% T = L.A(Q) is the 4x4 link homogeneous transformation matrix corresponding
% to the link variable Q which is either theta (revolute) or d (prismatic).
%
% Notes::
% - For a revolute joint the theta parameter of the link is ignored, and Q used instead.
% - For a prismatic joint the d parameter of the link is ignored, and Q used instead.
% - The link offset parameter is added to Q before computation of the transformation matrix.
	if L.mdh == 0
		T = linktran([L.alpha L.a L.theta L.d L.sigma], ...
			q+L.offset);
	else
		T = mlinktran([L.alpha L.a L.theta L.d L.sigma], ...
			q+L.offset);
	end
endfunction % A()

function t = linktran(a, b, c, d)
%LINKTRAN	Compute the link transform from kinematic parameters
%
%	LINKTRAN(alpha, an, theta, dn)
%	LINKTRAN(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the standard Denavit and Hartenberg notation.

%	Copright (C) Peter Corke 1993

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if sigma not given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st*ca	st*sa	an*ct
			st	ct*ca	-ct*sa	an*st
			0	sa	ca	dn
			0	0	0	1];
endfunction
%MLINKTRANS	Compute the link transform from kinematic parameters
%
%	MLINKTRANS(alpha, an, theta, dn)
%	MLINKTRANS(DH, q) is a homogeneous 
%	transformation between link coordinate frames.
%
%	alpha is the link twist angle
%	an is the link length
%	theta is the link rotation angle
%	dn is the link offset
%	sigma is 0 for a revolute joint, non-zero for prismatic
%
%	In the second case, q is substitued for theta or dn according to sigma.
%
%	Based on the modified Denavit and Hartenberg notation.

%	Copright (C) Peter Corke 1993
function t = mlinktrans(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if no sigma given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);

	t =    [	ct	-st	0	an
			st*ca	ct*ca	-sa	-sa*dn
			st*sa	ct*sa	ca	ca*dn
			0	0	0	1];
endfunction
			
function dyn(l)
%Link.dyn Display the inertial properties of link
%
% L.dyn() displays the inertial properties of the link object in a multi-line format.
% The properties shown are mass, centre of mass, inertia, friction, gear ratio
% and motor properties.
%
% If L is a vector of Link objects show properties for each element.

	if length(l) > 1
		for j=1:length(l)
			ll = l(j);
			fprintf('%d: %f; %f %f %f; %f %f %f\n', ...
				j, ll.m, ll.r, diag(ll.I));
			%dyn(ll);
		end
		return;
	end

	display(l);
	if ~isempty(l.m)
		fprintf('  m    = %f\n', l.m)
	end
	if ~isempty(l.r)
		fprintf('  r    = %f %f %f\n', l.r);
	end
	if ~isempty(l.I)
		fprintf('  I    = | %f %f %f |\n', l.I(1,:));
		fprintf('         | %f %f %f |\n', l.I(2,:));
		fprintf('         | %f %f %f |\n', l.I(3,:));
	end
	if ~isempty(l.Jm)
		fprintf('  Jm   = %f\n', l.Jm);
	end
	if ~isempty(l.B)
		fprintf('  Bm   = %f\n', l.B);
	end
	if ~isempty(l.Tc)
		fprintf('  Tc   = %f(+) %f(-)\n', l.Tc(1), l.Tc(2));
	end
	if ~isempty(l.G)
		fprintf('  G    = %f\n', l.G);
	end
	if ~isempty(l.qlim)
		fprintf('  qlim = %f to %f\n', l.qlim(1), l.qlim(2));
	end
endfunction % dyn()


