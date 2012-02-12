%QINTERP	Interpolate rotations expressed by quaternion objects
%
%	QI = qinterp(Q1, Q2, R)
%
%	Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
%	from 0 to 1.  This is a spherical linear interpolation (slerp) that can
%	be interpretted as interpolation along a great circle arc on a sphere.
%
%	If r is a vector, QI, is a cell array of quaternions.
%
%	See also TR2Q

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

% MOD HISTORY
% 2/99	convert to use of objects

%	Copright (C) Peter Corke 1999
function q = qinterp(Q1, Q2, r)


	q1 = double(Q1);
	q2 = double(Q2);

	if (r<0) | (r>1),
		error('R out of range');
	end

	theta = acos(q1*q2');
	q = {};
	count = 1;

	if length(r) == 1,
		q = quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
	else
		for R=r(:)',
			qq = quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
			q{count} = qq;
			count = count + 1;
		end
	end
