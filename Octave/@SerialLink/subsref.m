
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
function [v, a, b] = subsref(r, s)

	if s(1).type  ~= '.'
		%error('only .field supported')
	end

	% NOTE WELL:  the following code can't use getfield() since
	% getfield()  uses this, and Matlab will crash!!

	el = char(s(1).subs);
	switch el,
%-------------------------------------------------------------------------------------
% External Functions 	
	case 'fkine',
		q = s(2).subs;
		v = fkine(r,q{:});
       		
	case 'ikine6s',
		q = s(2).subs;
		v = ikine6s(r,q{:});
        			
	case 'ikine',
		q = s(2).subs;
		v = ikine(r,q{:});
	
	case 'maniplty',
		q = s(2).subs;
		[v, a] = maniplty(r,q{:});
		
	case 'rne'
		q = s(2).subs;
		[v, a] = rne(r,q{:});
		
	case 'isspherical',
		v = isspherical(r);
	
	case 'gravload'
		q = s(2).subs;
		v = gravload(r,q{:});
		
	case 'inertia'
		q = s(2).subs;
		v = inertia(r,q{:});
	
	case 'cinertia',
		q = s(2).subs;
		v = cinertia(r,q{:});
		
	case 'coriolis'
		q = s(2).subs;
		v = coriolis(r,q{:});
		
	case 'friction'
		q = s(2).subs;
		v = friction(r,q{:});
			
	case 'nofriction'
		if numel(s(2).subs) < 1 
		v = nofriction(r);
		else
		q = s(2).subs;
		v = nofriction(r,q{:});
		end
		
	case 'islimit',
		q = s(2).subs;
		v = islimit(r,q{:});
	
	case 'payload',
		q = s(2).subs;
		l = r.links;
		l(r.n).m = q{1};
		l(r.n).r = q{2};
		r.links = l;
		v = SerialLink(r);
		
	case 'jacob0',
		q = s(2).subs;
		v = jacob0(r,q{:});
		
	case 'accel',
		q = s(2).subs;
		v = accel(r,q{:});	
	
	case 'plot',
		q = s(2).subs;
		plot(r,q{:})	
		
	case 'fdyn',
		q = s(2).subs;
		[v, a, b] = fdyn(r,q{:});	
	
	case 'jacobn',
		q = s(2).subs;
		v = jacobn(r,q{:});	
		
	case 'jtraj',
		q = s(2).subs;
		v = jtraj(r,q{:});	
	
	case 'perturb',
		q = s(2).subs;
		v = perturb(r,q{:});	
	
	case 'itorque',
		q = s(2).subs;
		v = itorque(r,q{:});	
	
	case 'jacob_dot',
		q = s(2).subs;
		v = jacob_dot(r,q{:});	
	
	case 'dyn',
		q = s(2).subs;
		v = dyn(r,q{:});	
	
	case 'showlink',
		q = s(2).subs;
		v = showlink(r,q{:});	
	
%-------------------------------------------------------------------------------------	
	case 'links',
		
		if length(s) == 1,
			v = r.links;
			
		elseif length(s) == 2,
			if s(2).type == '()'
				j = s(2).subs;
				j = j{:};
				if (j < 1) | (j > r.n)
					error('link index out of bounds')
				end
			end
			v = r.links(j);
			
		elseif length(s) >= 3,
			if s(2).type == '()'
				j = s(2).subs;
				j = j{:};
				if (j < 1) | (j > r.n)
					error('link index out of bounds')
				end
			end
			if s(3).subs = 'dyn',
				link = r.links(j);
				v = link.dyn();
			end
		end
	case 'offset',
		L = r.links;
		v = [];
		for i=1:r.n,
			v = [v; L(i).offset];
		end
	case 'qlim',
		L = r.links;
		v = [];
		for i=1:r.n,
			v = [v; L(i).qlim];
		end
	
	case 'n',
		v = r.n;
	case 'name',
		v = r.name;
	case 'dh',
		v = rdh(r);
	case 'dyn'
		v = rdyn(r);
	case 'gravity'
		v = r.gravity;
	case 'tool'
		v = r.tool;
	case 'base'
		v = r.base;
	case 'mdh',
		v = r.mdh;
	case 'q',
		v = r.q;
	case 'plotopt',
		v = r.plotopt;
	case 'lineopt'
		v = r.lineopt;
	case 'shadowopt'
		v = r.shadowopt;
	case {'show', 'handle'}
		v = r.handle';
	otherwise, error('Unknown method in subsref')
	end
