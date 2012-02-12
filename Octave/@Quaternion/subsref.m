
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
function v = subsref(q, s)
	if (length (s)<2)
		if s(1).type  == '.' 

			% NOTE WELL:  the following code can't use getfield() since
			% getfield()  uses this, and Matlab will crash!!
						
			el = char(s(1).subs);
			switch el
			case 'd'
				v = double(q);
			case 's'
				v = q.s;
			case 'v'
				v = q.v;
			case 'T'
				v = q2tr(q);
			case 'R'
				v = q2tr(q);
				v = v(1:3,1:3);
			case 'inv'
				v = inv(q);
			case 'norm'
				v = norm(q);
			case 'unit'
				v = unit(q);
			case 'double'
				v = double(q);
			case 'plot'
				v = plot(q);
			end
		else
			error('only .field supported')
		end
			
	elseif (length(s) == 2 )
			if s(1).type  == '.' 
			
			% NOTE WELL:  the following code can't use getfield() since
			% getfield()  uses this, and Matlab will crash!!
						
			el = char(s(1).subs);
			args = s(2).subs;
				switch el
				case 'interp'
					v = interp(q,args{:});
				case 'scale'
					v = scale(q,args{:});
				case 'dot'
					v = dot(q,args{:});
				end
			else
			error('only .field supported')
			end
	else
		error('only .field supported')
	end
endfunction


