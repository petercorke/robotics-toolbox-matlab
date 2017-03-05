%SerialLink.rne Inverse dynamics
%
% TAU = R.rne(Q, QD, QDD, OPTIONS) is the joint torque required for the
% robot R to achieve the specified joint position Q (1xN), velocity QD
% (1xN) and acceleration QDD (1xN), where N is the number of robot joints.
%
% TAU = R.rne(X, OPTIONS) as above where X=[Q,QD,QDD] (1x3N).
%
% [TAU,WBASE] = R.rne(X, GRAV, FEXT) as above but the extra output is the
% wrench on the base.
%
% Options::
%  'gravity',G    specify gravity acceleration (default [0,0,9.81])
%  'fext',W       specify wrench acting on the end-effector W=[Fx Fy Fz Mx My Mz]
%  'slow'         do not use MEX file
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
%  - the 'slow' option is not given, and
%  - the robot is not symbolic, and
%  - the SerialLink property fast is true, and
%  - the MEX file frne.mexXXX exists in the subfolder rvctools/robot/mex.
%
% Notes::
% - The torque computed contains a contribution due to armature
%   inertia and joint friction.
% - See the README file in the mex folder for details on how to configure 
%   MEX-file operation.
% - The M-file is a wrapper which calls either RNE_DH or RNE_MDH depending on 
%   the kinematic conventions used by the robot object, or the MEX file.
% - If a model has no dynamic parameters set the result is zero.
%
% See also SerialLink.accel, SerialLink.gravload, SerialLink.inertia.

% TODO:
%  fix base transform 
%
% verified against MAPLE code, which is verified by examples
%



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


function varargout = rne(robot, varargin)
    
    % allow more descriptive arguments
    opt.gravity = [];
    opt.fext = [];
    opt.slow = false;
    
    [opt,args] = tb_optparse(opt, varargin);
    
    narg0 = length(args);
    if ~isempty(opt.gravity)
        assert( isvec(opt.gravity, 3), 'RTB:rne:badarg', 'gravity must be a 3-vector');
        args = [args opt.gravity(:)];
    end
    if ~isempty(opt.fext)
        assert( isvec(opt.fext, 6), 'RTB:rne:badarg', 'external force must be a 6-vector');
        if length(args) == 3
            args = [args robot.gravity];
        end
        args = [args opt.fext];
    end
    
    if robot.fast && ~opt.slow && (nargout < 2) && ~robot.issym() && ~any(cellfun( @(x) isa(x, 'sym'), args))
        % use the MEX-file implementation if:
        % * the fast property is set at constructor time
        % * slow override not set
        % * base wrench not requested
        % * robot has no symbolic parameters
        % * joint state has no symbolic values
        %
        % the mex-file handles DH and MDH variants
        
        % the MEX file doesn't handle base rotation, so we need to hack the gravity
        % vector instead.  It lives in the 4th element of args.
        
        if ~robot.base.isidentity()
            % ok, a non-identity transform, we need to fix it
            if length(args) == narg0 
                grav = robot.gravity;  % no gravity option provided, take default
            else
                grav = args{narg0+1}; % else take value from option processed above
            end
            args{narg0+1} = robot.base.R' * grav; % and put it in the argument list
        end

        [varargout{1:nargout}] = frne(robot, args{:});
    else
        % use the M-file implementation
        if robot.mdh == 0
            [varargout{1:nargout}] = rne_dh(robot, args{:});
        else
            [varargout{1:nargout}] = rne_mdh(robot, args{:});
        end
    end
