%SerialLink.IKUNC Numerical inverse manipulator without joint limits
%
% Q = R.ikunc(T) are the joint coordinates (1xN) corresponding to the robot 
% end-effector pose T (4x4) which is a homogenenous transform, and N is the
% number of robot joints.
%
% [Q,ERR] = robot.ikunc(T) as above but also returns ERR which is the
% scalar final value of the objective function.
%
% [Q,ERR,EXITFLAG] = robot.ikunc(T) as above but also returns the
% status EXITFLAG from fminunc.
%
% [Q,ERR,EXITFLAG] = robot.ikunc(T, Q0) as above but specify the
% initial joint coordinates Q0 used for the minimisation.
%
% [Q,ERR,EXITFLAG] = robot.ikunc(T, Q0, options) as above but specify the
% options for fminunc to use.
%
% Trajectory operation::
%
% In all cases if T is 4x4xM it is taken as a homogeneous transform
% sequence and R.ikunc() returns the joint coordinates corresponding to
% each of the transforms in the sequence.  Q is MxN where N is the number
% of robot joints. The initial estimate of Q for each time step is taken as
% the solution from the previous time step.
%
% ERR and EXITFLAG are also Mx1 and indicate the results of optimisation
% for the corresponding trajectory step.
%
% Notes::
% - Requires fminunc from the Optimization Toolbox.
% - Joint limits are not considered in this solution.
% - Can be used for robots with arbitrary degrees of freedom.
% - In the case of multiple feasible solutions, the solution returned
%   depends on the initial choice of Q0
% - Works by minimizing the error between the forward kinematics of the
%   joint angle solution and the end-effector frame as an optimisation.
%   The objective function (error) is described as:
%           sumsqr( (inv(T)*robot.fkine(q) - eye(4)) * omega )
%   Where omega is some gain matrix, currently not modifiable.
%
% Author::
% Bryan Moutrie
%
% See also SerialLink.ikcon, fmincon, SerialLink.ikine, SerialLink.fkine.

% Copyright (C) Bryan Moutrie, 2013-2015
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% LICENSE STATEMENT:
%
% This file is part of pHRIWARE.
% 
% pHRIWARE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as 
% published by the Free Software Foundation, either version 3 of 
% the License, or (at your option) any later version.
%
% pHRIWARE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU Lesser General Public 
% License along with pHRIWARE.  If not, see <http://www.gnu.org/licenses/>.


function [qstar, error, exitflag] = ikunc(robot, T, q0, options)

    % check if Optimization Toolbox exists, we need it
    if ~exist('fminunc')
        error('rtb:ikunc:nosupport', 'Optimization Toolbox required');
    end
    
    % create output variables
    T_sz = size(T,3);
    qstar = zeros(T_sz,robot.n);
    error = zeros(T_sz,1);
    exitflag = zeros(T_sz,1);
    
    problem.solver = 'fminunc';
    problem.x0 = zeros(1, robot.n);
    problem.options = optimoptions('fminunc', ...
        'Algorithm', 'quasi-newton', ...
        'Display', 'off'); % default options for ikunc
    
    if nargin > 2
        problem.x0 = q0;
    end
    if nargin > 3
        problem.options = optimset(problem.options, options);
    end
    
    reach = sum(abs([robot.a, robot.d]));
    omega = diag([1 1 1 3/reach]);
    
    for t = 1:T_sz
        problem.objective = ...
            @(x) sumsqr(((T(:,:,t) \ robot.fkine(x)) - eye(4)) * omega);
        
        [q_t, err_t, ef_t] = fminunc(problem);
        
        qstar(t,:) = q_t;
        error(t) = err_t;
        exitflag(t) = ef_t;
        
        problem.x0 = q_t;
    end
    
end

function s = sumsqr(A)
    s = sum(A(:).^2);
end

