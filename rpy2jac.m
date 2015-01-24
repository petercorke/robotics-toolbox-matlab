%RPY2JAC Jacobian from RPY angle rates to angular velocity
%
% J = RPY2JAC(EUL) is a Jacobian matrix (3x3) that maps roll-pitch-yaw angle 
% rates to angular velocity at the operating point RPY=[R,P,Y].
%
% J = RPY2JAC(R, P, Y) as above but the roll-pitch-yaw angles are passed
% as separate arguments.
%
% Notes::
% - Used in the creation of an analytical Jacobian.
%
% See also EUL2JAC, SerialLink.JACOBN.



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

function J = rpy2jac(r, p, y)

    if length(r) == 3
        % rpy2jac([r,p,y])
        p = r(2);
        y = r(3);
        r = r(1);
    elseif nargin ~= 3
        error('RTB:rpy2jac:badarg', 'bad arguments');
    end
	J = [	
        1  0       sin(p)
        0  cos(r)  -cos(p)*sin(r)
        0  sin(r)  cos(p)*cos(r)
        ];
		
