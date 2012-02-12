
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
function r = subsasgn(r, s, v)

	if s(1).type  ~= '.'
		error('only .field supported')
	end
	switch s(1).subs,
	case 'links'
		r.links = v;
	case 'lineopt',
		r.lineopt = v;
	case 'shadowopt',
		r.shadowopt = v;
	case 'offset',
		L = r.links;
		for i=1:r.n,
			L(i).offset = v(i);
		end
	case 'qlim',
		if numrows(v) ~= r.n,
			error('insufficient rows in joint limit matrix');
		end
		L = r.links;
		for i=1:r.n,
			L(i).qlim = v(i,:);
		end
	case 'q',
		r.q = v;
	case 'handle',
		r.handle = v;
	case 'plotopt',
		r.plotopt = v;
	case 'gravity',
		r.gravity = v;
	case 'tool',
		if ~ishomog(v)
			error('base must be a homogeneous transform');
		end
		r.tool = v;
	case 'base',
		if ~ishomog(v)
			error('base must be a homogeneous transform');
		end
		r.base = v;
	case 'name',
		r.name = v;
	case 'manufacturer',
		r.manufacturer = v;
	case 'comment',
		r.comment = v;
	otherwise, error('Unknown method in subsasgn')
	end
