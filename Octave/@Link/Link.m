%LINK create a new LINK object
%
% A LINK object holds all information related to a robot link such as
% kinematics of the joint, rigid-body inertial parameters, motor and
% transmission parameters.
%
%	LINK
%	LINK(link)
%
%	Create a default link, or a clone of the passed link.
%
%	A = LINK(q)
%
%	Compute the link transform matrix for the link, given the joint
%	variable q.
%
%	LINK([alpha A theta D sigma])
%	LINK(DH_ROW)	create from row of legacy DH matrix
%	LINK(DYN_ROW)	create from row of legacy DYN matrix
%
% Any of the last 3 forms can have an optional flag argument which is 0
% for standard D&H parameters and 1 for modified D&H parameters.
% Handling the different kinematic conventions is now hidden within the LINK
% object.
%
% Conceivably all sorts of stuff could live in the LINK object such as
% graphical models of links and so on.

% MOD HISTORY
% 3/99	modify to use on a LINK object
% 6/99	fix the number of fields inthe object, v5.3 doesn't let me change them
%	mod by  Francisco Javier Blanco Rodriguez <jblanco@abedul.usal.es>


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

function l = Link(dh, convention)

	% legacy DH matrix
	% link([theta d a alpha])
	% link([theta d a alpha sigma])
	% link([theta d a alpha sigma offset])

	if nargin == 0,
		l.theta = 0;
		l.d = 0;
		l.a = 0;
		l.alpha = 0;
		l.sigma = 0;
		l.offset = 0;
		l.mdh = 0;
		% it's a legacy DYN matrix
		l.m = [];
		l.r = [];
		v = [];
		l.I = [];
		l.Jm = [];
		l.G = [];
		l.B = 0;
		l.Tc = [0 0];
		l.qlim = [];
		l = class(l, "Link");
	

	elseif isa(dh, 'Link')
		l = dh;
	elseif length(dh) < 4
			error('must provide params (theta d a alpha)');
	elseif length(dh) <= 6
		% legacy DH matrix
		l.theta = dh(1);
		l.d = dh(2);
		l.a = dh(3);
		l.alpha = dh(4);
		
		
		if length(dh) >= 5,
			l.sigma = dh(5);
		else 
			l.sigma = 0;
		end
		if length(dh) >= 6
			l.offset = dh(6);
		else 
			l.offset = 0;
		end
		l.mdh = 0;
		if nargin > 1
			if strncmp(convention, 'mod', 3) == 1
				l.mdh = 1;
			elseif strncmp(convention, 'sta', 3) == 1
				l.mdh = 0;
			else
				error('convention must be modified or standard');
			end
		end
		
		% we know nothing about the dynamics
		l.m = [];
		l.r = [];
		v = [];
		l.I = [];
		l.Jm = [];
		l.G = [];
		l.B = 0;
		l.Tc = [0 0];
		l.qlim = [];
		l = class(l, "Link");
		
	
	else
		% legacy DYN matrix

		l.theta = dh(1);
		l.d = dh(2);
		l.a = dh(3);
		l.alpha = dh(4);
		if length(dh) >= 5,
			l.sigma = dh(5);
		else
			l.sigma = 0;
		end
		l.offset = 0;
		l.mdh = 0;
		if nargin > 1
			if strncmp(convention, 'mod', 3) == 1
				l.mdh = 1;
			elseif strncmp(convention, 'sta', 3) == 1
				l.mdh = 0;
			else
				error('convention must be modified or standard');
			end
		end
		% it's a legacy DYN matrix
		if length(dh) >= 6,
			l.m = [dh(6)];
		else
			l.m = [];
		end
		if length(dh) >= 9,
			l.r = dh(7:9);
		else
			l.r = [];
		end
		if length(dh) >= 15,
			v = dh(10:15);
			l.I = [	v(1) v(4) v(6)
					v(4) v(2) v(5)
					v(6) v(5) v(3)];
		else
			v = [];
			l.I = [];
		end
		if length(dh) >= 16,
			l.Jm = dh(16);
		else
			l.Jm = [];
		end
		if length(dh) >= 17,
			l.G = dh(17);
		else
			l.G = [];
		end
		if length(dh) >= 18,
			l.B = dh(18);
		else
			l.B = 0;
		end
		if length(dh) >= 20,
			l.Tc = dh(19:20);
		else
			l.Tc = [0 0];
		end
		
		l.qlim = [];
		l = class(l, "Link");
		
	end
	
endfunction 
