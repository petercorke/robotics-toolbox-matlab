%SerialLink.fdyn Integrate forward dynamics
%
% [T,Q,QD] = R.fdyn(TMAX, FTFUN) integrates the dynamics of the robot over
% the time  interval 0 to TMAX and returns vectors of time T (Kx1), joint
% position Q (KxN) and joint velocity QD (KxN).  The initial joint position
% and velocity are zero. The torque applied to the joints is computed by
% the user-supplied control function FTFUN:
%
%        TAU = FTFUN(ROBOT, T, Q, QD)
%
% where TAU (1xN) is the joint torques to be applied at this time instant,
% ROBOT is the robot object being simulated, T is the current time, and Q
% (1xN) and QD (1xN) are the manipulator joint coordinate and velocity
% state respectively.
%
% [TI,Q,QD] = R.fdyn(T, FTFUN, Q0, QD0) as above but allows the initial
% joint position Q0 (1xN) and velocity QD0 (1x)  to be specified.
%
% [T,Q,QD] = R.fdyn(T1, FTFUN, Q0, QD0, ARG1, ARG2, ...) allows optional 
% arguments to be passed through to the user-supplied control function:
%
%        TAU = FTFUN(ROBOT, T, Q, QD, ARG1, ARG2, ...)
%
% Example 1: to apply zero joint torque to the robot without Coulomb friction:
%
%          [t,q] = robot.nofriction().fdyn(5, @my_torque_function);
%
%          function tau = my_torque_function(robot, t, q)
%              tau = zeros(1, robot.n);
%          end
%
% Example 2: as above, but using a lambda function
%
%          [t,q] = robot.nofriction().fdyn(5, @(robot, t, q, qd) zeros(1, robot.n) );
%
%
% Example 3: the robot is controlled by a PD controller. We first define
% a function to compute the control which has additional parameters for
% the setpoint and control gains (qstar, P, D)
%
%         function tau = myfunc(q, qd, qstar, P, D)
%             tau = (qstar-q)*P + qd*D;  % P, D are 6x6
%         end
%
% and then integrate the robot dynamics with the control:
%
%         [t,q] = robot.fdyn(10, @(robot, t, q, qd) myfunc(q, qd, qstar, P, D) );
%
% where the lambda function ignores the passed values of robot and t but
% adds qstar, P and D to argument list for myfunc.
%
% Note::
% - This function performs poorly with non-linear joint friction, such as
%   Coulomb friction.  The R.nofriction() method can be used to set this 
%   friction to zero.
% - If FTFUN is not specified, or is given as [],  then zero torque
%   is applied to the manipulator joints.
% - The MATLAB builtin integration function ode45() is used.
%
% See also SerialLink.accel, SerialLink.nofriction, SerialLink.rne, ode45.


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

function [t, q, qd] = fdyn(robot, t1, torqfun, q0, qd0, varargin)

    % check the Matlab version, since ode45 syntax has changed
    if verLessThan('matlab', '7')  
        error('fdyn now requires MATLAB version >= 7');
    end

    
    n = robot.n;
    if nargin == 2
        torqfun = [];
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 3
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 4
        qd0 = zeros(1,n);
    end
    
    assert(isempty(torqfun) || isa(torqfun,'function_handle') , 'RTB:fdyn:badarg', 'must pass a function handle or []');


    % concatenate q and qd into the initial state vector
    q0 = [q0(:); qd0(:)];
        
    [t,y] = ode45(@fdyn2, [0 t1], q0, [], robot, torqfun, varargin{:});
    q = y(:,1:n);
    qd = y(:,n+1:2*n);

end


%FDYN2  private function called by FDYN
%
%   XDD = FDYN2(T, X, FLAG, ROBOT, TORQUEFUN)
%
% Called by FDYN to evaluate the robot velocity and acceleration for
% forward dynamics.  T is the current time, X = [Q QD] is the state vector,
% ROBOT is the object being integrated, and TORQUEFUN is the string name of
% the function to compute joint torques and called as
%
%       TAU = TORQUEFUN(ROBOT, T, X, VARARGIN)
%
% if not given zero joint torques are assumed.
%
% The result is XDD = [QD QDD].
function xd = fdyn2(t, x, robot, torqfun, varargin)

    n = robot.n;

    q = x(1:n)';
    qd = x(n+1:2*n)';

    % evaluate the torque function if one is given
    if isa(torqfun, 'function_handle')
        tau = torqfun(robot, t, q, qd, varargin{:});
        tau = tau(:)';
        assert(isreal(tau) && length(tau) == n, 'rtb:fdyn:badretval', 'torque function must return vector with N real elements');
    else
        tau = zeros(1,n);
    end
    
    qdd = robot.accel(x(1:n,1)', x(n+1:2*n,1)', tau);
    xd = [x(n+1:2*n,1); qdd];
end
