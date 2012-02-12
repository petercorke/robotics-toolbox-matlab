%PLOT	plot a quaternion object as a rotated coordinate frame

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

%	Copright (C) Peter Corke 1999
function plot(Q)
	axis([-1 1 -1 1 -1 1])


	o = [0 0 0]';
	x1 = Q*[1 0 0]';
	y1 = Q*[0 1 0]';
	z1 = Q*[0 0 1]';

	hold on
	plot3([0;x1(1)], [0; x1(2)], [0; x1(3)])
	text(x1(1), x1(2), x1(3), 'X')
	plot3([0;y1(1)], [0; y1(2)], [0; y1(3)])
	text(y1(1), y1(2), y1(3), 'Y')
	plot3([0;z1(1)], [0; z1(2)], [0; z1(3)])
	text(z1(1), z1(2), z1(3), 'Z')
	grid on
	xlabel('X')
	ylabel('Y')
	zlabel('Z')
	hold off
