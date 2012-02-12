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
function s = char(links, from_robot)
        %Link.char String representation of parameters
        %
        % s = L.char() is a string showing link parameters in compact single line format.  
        % If L is a vector of Link objects return a string with one line per Link.
        %
        % See also Link.display.

            % display in the order theta d a alpha
            if nargin < 2
                from_robot = false;
            end

            s = '';
            for j=1:length(links)
                l = links(j);
                if from_robot
                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf('|%3d|%11.4g|%11s|%11.4g|%11.4g|', ...
                            j, l.theta, sprintf('q%d', j), l.a, l.alpha);
                    else
                        js = sprintf('|%3d|%11s|%11.4g|%11.4g|%11.4g|', ...
                            j, sprintf('q%d', j), l.d, l.a, l.alpha);
                    end
                else
                    if l.sigma == 0,
						conv = 'R';
					else
						conv = 'P';
					end
                    if l.mdh == 0
                        conv = [conv ',stdDH'];
                    else
                        conv = [conv ',modDH'];
                    end
                    if length(links) == 1
                        qname = 'q';
                    else
                        qname = sprintf('q%d', j);
                    end

                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf(' theta=%.4g, d=%s, a=%.4g, alpha=%.4g (%s)', ...
                            l.theta, qname, l.a, l.alpha, conv);
                    else
                        js = sprintf(' theta=%s, d=%.4g, a=%.4g, alpha=%.4g (%s)', ...
                            qname, l.d, l.a, l.alpha, conv);
                    end
                end
                s = strvcat(s, js);
            end
        end % char()
