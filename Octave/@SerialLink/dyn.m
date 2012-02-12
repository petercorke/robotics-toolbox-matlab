
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
function dyn(r)
%SerialLink.dyn Display inertial properties
%
% R.dyn() displays the inertial properties of the SerialLink object in a multi-line 
% format.  The properties shown are mass, centre of mass, inertia, gear ratio, 
% motor inertia and motor friction.
%
% See also Link.dyn.
	for j=1:r.n
		fprintf('----- link %d\n', j);
		l = r.links(j);
		l.dyn()
	end
end
