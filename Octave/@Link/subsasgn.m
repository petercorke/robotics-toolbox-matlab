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
function this = subsasgn(this, index, value)
	
	switch index(1).type 
	case '.'
		switch index(1).subs,
		case 'alpha',
			this.alpha = value;
		case 'a',
			this.a = value;
		case 'theta',
			this.theta = value;
		case 'd',
			this.d = value;
		case 'offset',
			this.offset = value;
		case 'sigma',
			if ischar(value)
				this.sigma = lower(value) == 'p';
			else
				this.sigma = value;
			end
		case 'mdh',
			this.mdh = value;
		case 'G',
			this.G = value;
		case 'I',
			if isempty(value)
				return;
			end
			if all(size(value) == [3 3])
				if norm(value-value') > eps
					error('inertia matrix must be symmetric');
				end
				this.I = value;
			elseif length(value) == 3
				this.I = diag(value);
			elseif length(value) == 6
				this.I = [ value(1) value(4) value(6)
					value(4) value(2) value(5)
					value(6) value(5) value(3)];
			end
		case 'r',
			if isempty(value)
				return;
			end
			if length(value) ~= 3
				error('COG must be a 3-vector');
			end
			this.r = value(:)';
		case 'Jm',
			this.Jm = value;
		case 'B',
			this.B = value;
		case 'Tc',
			if isempty(value)
				return;
			end
			if length(value) == 1
				this.Tc = [value -value];
			elseif length(value) == 2
				if value(1) < value(2)
					error('Coulomb friction is [Tc+ Tc-]');
				end
				this.Tc = value;
			else
				error('Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only')
			end
		case 'm',
			this.m = value;
		case 'qlim',
			if length(value) ~= 2,
				error('joint limit must have 2 elements');
			end
			this.qlim = value;
                 
        
		otherwise, error('Unknown method')
		end
	
	case '()'
		if numel(index) == 1
			if isempty(this)
			this  = value; %% this is a crude bug fix
			end 
			
			this = builtin('subsasgn', this, index, value);
		else
			this_subset = this(index(1).subs{:}); % get the subset

			this_subset = subsasgn(this_subset, index(2:end), value);
			
			this(index(1).subs{:}) = this_subset;  % put subset back;
		end
	end
endfunction	
