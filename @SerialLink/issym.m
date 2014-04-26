%SerialLink.issym Check if Link or SerialLink object is a symbolic model
%
% res = L.issym() is true if the Link L has symbolic parameters.
%
% res = R.issym() is true if the SerialLink manipulator R has symbolic parameters
%
%
% Authors::
% Jörn Malzahn   
% 2012 RST, Technische Universität Dortmund, Germany
% http://www.rst.e-technik.tu-dortmund.de    

% Copyright (C) 1993-2014, by Peter I. Corke
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
function res = issym(l)

	res = issym(l.links(1));
	
end