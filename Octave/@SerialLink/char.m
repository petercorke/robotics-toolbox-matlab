
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
function s = char(robot)
%
% S = R.char() is a string representation of the robot parameters.

	s = '';
	for j=1:length(robot)
		r = robot(j);

		% informational line
		if r.mdh
			convention = 'modDH';
		else
			convention = 'stdDH';
		end
		s = sprintf('%s (%d axis, %s, %s)', r.name, r.n, config(r), convention);

		% comment and other info
		line = '';
		if ~isempty(r.manufacturer)
			line = strcat(line, sprintf(' %s;', r.manufacturer));
		end
		if ~isempty(r.comment)
			line = strcat(line, sprintf(' %s;', r.comment));
		end
		s = strvcat(s, line);

		% link parameters
		s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');
		s = strvcat(s, '| j |     theta |         d |         a |     alpha |');
		s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');
		s = strvcat(s, char(r.links, true));
		s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');

		% gravity, base, tool
		
		s_grav = horzcat(strvcat('grav = ', ' ', ' '), num2str(r.gravity));
		s_grav = strvcat(s_grav, " ");
		s_base = horzcat(strvcat("  base = ",' ',' ', ' '),num2str(r.base));

		s_tool = horzcat(strvcat('   tool =  ',' ',' ', ' '), num2str(r.tool));
		
		line = horzcat(s_grav, s_base, s_tool);

		s = strvcat(s, ' ', line);
		if j ~= length(robot)
			s = strvcat(s, ' ');
		end
	end
endfunction

function v = config(r)
            v = '';
            for i=1:r.n
                v(i) = r.links(i).RP;
            end
        end
