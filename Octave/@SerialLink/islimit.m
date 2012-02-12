
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
function v = islimit(r,q)
%SerialLink.islimit Joint limit test
%
% V = R.ISLIMIT(Q) is a vector of boolean values, one per joint, 
% false (0) if Q(i) is within the joint limits, else true (1).
	L = r.links;
	if length(q) ~= r.n
		error('argument for islimit method is wrong length');
	end
	v = [];
	for i=1:r.n
		v = [v; r.links(i).islimit(q(i))];
	end
end
