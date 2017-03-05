%SerialLink.perturb Perturb robot parameters
%
% RP = R.perturb(P) is a new robot object in which the dynamic parameters (link
% mass and inertia) have been perturbed.  The perturbation is multiplicative so 
% that values are multiplied by random numbers in the interval (1-P) to (1+P).
% The name string of the perturbed robot is prefixed by 'P/'.
%
% Useful for investigating the robustness of various model-based control 
% schemes. For example to vary parameters in the range +/- 10 percent is:
%    r2 = p560.perturb(0.1);
%
% See also SerialLink.rne.


% Copyright (C) 1993-2017, by Peter I. Corke
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

function  r2 = perturb(r, p)

	if nargin == 1
		p = 0.1;	% 10 percent disturb by default
	end

    r2 = SerialLink(r);

    links = r2.links;
	for i=1:r.n
		s = (2*rand-1)*p + 1;
		links(i).m = links(i).m * s;

		s = (2*rand-1)*p + 1;
		links(i).I = links(i).I * s;
	end

	r2.name = ['P/' r.name];
