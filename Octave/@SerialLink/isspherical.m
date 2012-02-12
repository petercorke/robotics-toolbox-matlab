
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
function v = isspherical(r)
%SerialLink.isspherical Test for spherical wrist
%
% R.isspherical() is true if the robot has a spherical wrist, that is, the 
% last 3 axes intersect at a point.
%
% See also SerialLink.ikine6s.
	L = r.links(end-2:end);

	v = false;
	if ~isempty( find( [L(1).a L(2).a L(3).a L(2).d L(3).d] ~= 0 ))
		return
	end

	if (abs(L(1).alpha) == pi/2) & (abs(L(1).alpha + L(2).alpha) < eps)
		v = true;
		return;
	end
end
        
