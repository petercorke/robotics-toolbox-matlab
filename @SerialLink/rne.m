%SerialLink.rne Inverse dynamics
%
% TAU = R.rne(Q, QD, QDD) is the joint torque required for the robot R to
% achieve the specified joint position Q (1xN), velocity QD (1xN) and
% acceleration QDD (1xN), where N is the number of robot joints.
%
% TAU = R.rne(Q, QD, QDD, GRAV) as above but overriding the gravitational 
% acceleration vector (3x1) in the robot object R.
%
% TAU = R.rne(Q, QD, QDD, GRAV, FEXT) as above but specifying a wrench 
% acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
%
% TAU = R.rne(X) as above where X=[Q,QD,QDD] (1x3N).
%
% TAU = R.rne(X, GRAV) as above but overriding the gravitational 
% acceleration vector in the robot object R.
%
% TAU = R.rne(X, GRAV, FEXT) as above but specifying a wrench 
% acting on the end of the manipulator which is a 6-vector [Fx Fy Fz Mx My Mz].
%
% [TAU,WBASE] = R.rne(X, GRAV, FEXT) as above but the extra output is the
% wrench on the base.
%
% Trajectory operation::
%
% If Q,QD and QDD (MxN), or X (Mx3N) are matrices with M rows representing a 
% trajectory then TAU (MxN) is a matrix with rows corresponding to each trajectory 
% step.
%
% MEX file operation::
% This algorithm is relatively slow, and a MEX file can provide better
% performance.  The MEX file is executed if:
%  - the robot is not symbolic, and
%  - the SerialLink property fast is true, and
%  - the MEX file frne.mexXXX exists in the subfolder rvctools/robot/mex.
%
% Notes::
% - The robot base transform is ignored.
% - Currently the MEX-file version does not compute WBASE.
% - The torque computed contains a contribution due to armature
%   inertia and joint friction.
% - See the README file in the mex folder for details on how to configure 
%   MEX-file operation.
% - The M-file is a wrapper which calls either RNE_DH or RNE_MDH depending on 
%   the kinematic conventions used by the robot object, or the MEX file.
%
% See also SerialLink.accel, SerialLink.gravload, SerialLink.inertia.

% TODO:
% should use tb_optparse
%
% verified against MAPLE code, which is verified by examples
%



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


function varargout = rne(robot, varargin)

    if robot.fast && ~robot.issym()
        % use the MEX-file implementation
        [varargout{1:nargout}] = frne(robot, varargin{:});
    else
        % use the M-file implementation
        if robot.mdh == 0
            [varargout{1:nargout}] = rne_dh(robot, varargin{:});
        else
            [varargout{1:nargout}] = rne_mdh(robot, varargin{:});
        end
    end
