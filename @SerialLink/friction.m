%SerialLink.friction Friction force
%
% TAU = R.friction(QD) is the vector of joint friction forces/torques for the 
% robot moving with joint velocities QD.  
%
% The friction model includes:
% - Viscous friction which is a linear function of velocity.
% - Coulomb friction which is proportional to sign(QD).
%
% Notes::
% - The friction value should be added to the motor output torque, it has a
%   negative value when QD>0. 
% - The returned friction value is referred to the output of the gearbox.
% - The friction parameters in the Link object are referred to the motor.
% - Motor viscous friction is scaled up by G^2.
% - Motor Coulomb friction is scaled up by G.
% - The appropriate Coulomb friction value to use in the non-symmetric case
%   depends on the sign of the joint velocity, not the motor velocity.
% - The absolute value of the gear ratio is used.  Negative gear ratios are
%   tricky: the Puma560 has negative gear ratio for joints 1 and 3.
%
% See also Link.friction.



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

function  tau = friction(robot, qd)

	L = robot.links;

    tau = zeros(1,robot.n);
    if robot.issym
        tau = sym(tau);
    end
    
    
	for j=1:robot.n
		tau(j) = L(j).friction(qd(j));
	end

