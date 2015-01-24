%SerialLink.nofriction Remove friction 
%
% RNF = R.nofriction() is a robot object with the same parameters as R but 
% with non-linear (Coulomb) friction coefficients set to zero.  
%
% RNF = R.nofriction('all') as above but viscous and Coulomb friction coefficients set to zero.
%
% RNF = R.nofriction('viscous') as above but viscous friction coefficients 
% are set to zero.
%
% Notes::
% - Non-linear (Coulomb) friction can cause numerical problems when integrating
%   the equations of motion (R.fdyn).
% - The resulting robot object has its name string prefixed with 'NF/'.
%
% See also SerialLink.fdyn, Link.nofriction.




% Copyright (C) 1993-2015, by Peter I. Corke
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

function  r2 = nofriction(r, varargin)

	r2 = SerialLink(r); % make a copy

	for j=1:r2.n
		r2.links(j) = r.links(j).nofriction(varargin{:});
	end

	r2.name = ['NF/' r.name];
