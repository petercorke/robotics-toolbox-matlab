%SerialLink.fdyn Integrate forward dynamics
%
% [T,Q,QD] = R.fdyn(T1, TORQFUN) integrates the dynamics of the robot over 
% the time  interval 0 to T and returns vectors of time TI, joint position Q
% and joint velocity QD.  The initial joint position and velocity are zero.
% The torque applied to the joints is computed by the user function TORQFUN:
%
% [TI,Q,QD] = R.fdyn(T, TORQFUN, Q0, QD0) as above but allows the initial
% joint position and velocity to be specified.
%
% The control torque is computed by a user defined function
%
% 	TAU = TORQFUN(T, Q, QD, ARG1, ARG2, ...)
%
% where Q and QD are the manipulator joint coordinate and velocity state 
% respectively], and T is the current time. 
%
% [T,Q,QD] = R.fdyn(T1, TORQFUN, Q0, QD0, ARG1, ARG2, ...) allows optional 
% arguments to be passed through to the user function.
%
% Note::
% - This function performs poorly with non-linear joint friction, such as
%   Coulomb friction.  The R.nofriction() method can be used to set this 
%   friction to zero.
% - If TORQFUN is not specified, or is given as 0 or [],  then zero torque
%   is applied to the manipulator joints.
% - The builtin integration function ode45() is used.
%
% See also SerialLink.accel, SerialLink.nofriction, SerialLink.RNE, ode45.

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

function [t, q, qd] = fdyn(robot, t1, torqfun, q0, qd0, varargin)
    global __fdyn_robot;
    global __fdyn_torqfun;

	n = robot.n;
	if nargin == 2
		torqfun = 0;
		q0 = zeros(1,n);
		qd0 = zeros(1,n);
	elseif nargin == 3
		q0 = zeros(1,n);
		qd0 = zeros(1,n);
	elseif nargin == 4
		qd0 = zeros(1,n);
	end

    % concatenate q and qd into the initial state vector
	x0= [q0(:); qd0(:)];
	
    __fdyn_robot = robot;
    __fdyn_torqfun = torqfun;
	
	dt = t1/100;
	t = 0:dt:t1;
	
	y = lsode(@fdyn2, x0, t);
	t = t'
	q = y(:,1:n);
	qd = y(:,n+1:2*n);


%FDYN2  private function called by FDYN
%
%	XDD = FDYN2(T, X, FLAG, ROBOT, TORQUEFUN)
%
% Called by FDYN to evaluate the robot velocity and acceleration for
% forward dynamics.  T is the current time, X = [Q QD] is the state vector,
% ROBOT is the object being integrated, and TORQUEFUN is the string name of
% the function to compute joint torques and called as
%
%       TAU = TORQUEFUN(T, X)
%
% if not given zero joint torques are assumed.
%
% The result is XDD = [QD QDD].


function xd = fdyn2(x, t)
    global __fdyn_robot;
	global __fdyn_torqfun;
    robot = __fdyn_robot;
    torqfun = __fdyn_torqfun;
	n = robot.n;
	q = x(1:n);
	qd = x(n+1:2*n);
	if isstr(torqfun)
		tau = feval(torqfun, t, q, qd);
	else
		tau = zeros(n,1)';
	end
		
	qdd = accel(robot, x(1:n,1)', x(n+1:2*n,1)', tau);
	xd = [x(n+1:2*n,1); qdd];
	
	
