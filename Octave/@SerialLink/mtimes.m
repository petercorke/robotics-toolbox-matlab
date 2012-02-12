%	robot objects can be multiplied r1*r2 which is mechanically equivalent
%	to mounting robot r2 on the end of robot r1.
%

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
function r2 = mtimes(r, l)
        %SerialLink.mtimes Join robots
        %
        % R = R1 * R2 is a robot object that is equivalent to mounting robot R2 
        % on the end of robot R1.
            if isa(l, 'SerialLink')
                r2 = SerialLink(r);
				x = r2.links;
				for i = 1:length(x) 
					new_links(i) = Link(x(i));
				end 
				y = l.links;
				for i = 1:length(y)
					new_links(i + length(x)) = Link(y(i));
				end
				r2.links = new_links;
                r2.base = r.base;
                r2.n = length(r2.links);
            elseif isa(l, 'Link')
                r2 = SerialLink(r);
				x = r2.links;
				for i = 1:length(x) 
					new_links(i) = Link(x(i));
				end 
				for i = 1:length(l)
					new_links(i + length(x)) = Link(l(i));
				end
				
                r2.links = new_links;
                r2.n = length(r2.links);
            end
        end
