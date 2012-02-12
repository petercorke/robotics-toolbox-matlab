%FRICTION	compute friction torque on the LINK object
%
%	TAU = FRICTION(LINK, QD)
%
%	Return the friction torque on the link moving at speed QD.  Depending
%	on fields in the LINK object viscous and/or Coulomb friction
%	are computed.
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

function  tau = friction(l, qd)
%Link.friction Joint friction force
%
% F = L.friction(QD) is the joint friction force/torque for link velocity QD
	tau = 0.0;

	tau = l.B * qd;

	if qd > 0
		tau = tau + l.Tc(1);
	elseif qd < 0
		tau = tau + l.Tc(2);
	end
	tau = -tau;     % friction opposes motion
endfunction % friction()
