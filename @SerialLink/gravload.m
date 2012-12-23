%SerialLink.gravload Gravity loading
%
% TAUG = R.gravload(Q) is the joint gravity loading for the robot in the
% joint configuration Q.  Gravitational acceleration is a property of the
% robot object.
%
% If Q is a row vector, the result is a row vector of joint torques.  If 
% Q is a matrix, each row is interpreted as a joint configuration vector, 
% and the result is a matrix each row being the corresponding joint torques.
%
% TAUG = R.gravload(Q, GRAV) is as above but the gravitational 
% acceleration vector GRAV is given explicitly.
%
% See also SerialLink.rne, SerialLink.itorque, SerialLink.coriolis.



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function tg = gravload(robot, q, grav)
	if numcols(q) ~= robot.n
		error('Insufficient columns in q')
	end
	if nargin == 2
		tg = rne(robot, q, zeros(size(q)), zeros(size(q)));
	elseif nargin == 3
		tg = rne(robot, q, zeros(size(q)), zeros(size(q)), grav);
    end
