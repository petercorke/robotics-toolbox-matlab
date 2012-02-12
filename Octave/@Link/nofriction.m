%NOFRICTION	return link object with zero friction 
%
%	LINK = NOFRICTION(LINK)
%
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
function  l2 = nofriction(l, only)
%Link.nofriction Remove friction
%
% LN = L.nofriction() is a link object with the same parameters as L except 
% nonlinear (Coulomb) friction parameter is zero.
%
% LN = L.nofriction('all') is a link object with the same parameters as L 
% except all friction parameters are zero.

	l2 = Link(l);
	if (nargin == 2) && strcmpi(only(1:3), 'all')
		l2.B = 0;
	end
	l2.Tc = [0 0];
end
