
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
function jt = jtraj(r, T1, T2, t, varargin)
%SerialLink.jtraj Create joint space trajectory
%
% Q = R.jtraj(T0, TF, M) is a joint space trajectory where the joint
% coordinates reflect motion from end-effector pose T0 to TF in M steps  with 
% default zero boundary conditions for velocity and acceleration.  
% The trajectory Q is an MxN matrix, with one row per time step, and 
% one column per joint, where N is the number of robot joints.
%
% Note::
% - requires solution of inverse kinematics. R.ikine6s() is used if
%   appropriate, else R.ikine().  Additional trailing arguments to R.jtraj()
%   are passed as trailing arugments to the these functions.
%
% See also jtraj, SerialLink.ikine, SerialLink.ikine6s.
	if isspherical(r) && (r.n == 6)
		q1 = ikine6s(r, T1, varargin{:});
		q2 = ikine6s(r, T2, varargin{:});
	else
		q1 = ikine(r, T1, varargin{:});
		q2 = ikine(r, T2, varargin{:});
	end
	
	jt = jtraj(q1, q2, t);
end
