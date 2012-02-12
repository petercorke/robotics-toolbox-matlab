%SerialLink.rne Inverse dynamics
%
% TAU = R.rne(Q, QD, QDD) is the joint torque required for the robot R
% to achieve the specified joint position Q, velocity QD and acceleration QDD.
%
% TAU = R.rne(Q, QD, QDD, GRAV) as above but overriding the gravitational 
% acceleration vector in the robot object R.
%
% TAU = R.rne(Q, QD, QDD, GRAV, FEXT) as above but specifying a wrench 
% acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
%
% TAU = R.rne(X) as above where X=[Q,QD,QDD].
%
% TAU = R.rne(X, GRAV) as above but overriding the gravitational 
% acceleration vector in the robot object R.
%
% TAU = R.rne(X, GRAV, FEXT) as above but specifying a wrench 
% acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
%
% If Q,QD and QDD, or X are matrices with M rows representing a trajectory then
% TAU is an MxN matrix with rows corresponding to each trajectory state.
%
% Notes:
% - The robot base transform is ignored
% - The torque computed also contains a contribution due to armature
%   inertia.
% - RNE can be either an M-file or a MEX-file.  See the manual for details on
%   how to configure the MEX-file.  The M-file is a wrapper which calls either
%   RNE_DH or RNE_MDH depending on the kinematic conventions used by the robot
%   object.
%
% See also SerialLink.accel, SerialLink.gravload, SerialLink.inertia.

% TODO:
% should use tb_optparse
%
% verified against MAPLE code, which is verified by examples
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


function [tau,f] = rne(robot, varargin)
	if robot.mdh == 0,
		[tau,f] = rne_dh(robot, varargin{:});
	else
		tau = rne_mdh(robot, varargin{:});
	end
endfunction 
