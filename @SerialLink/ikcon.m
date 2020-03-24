%SerialLink.IKCON Inverse kinematics by optimization with joint limits
%
% Q = R.ikcon(T, OPTIONS) are the joint coordinates (1xN) corresponding to
% the robot end-effector pose T which is an SE3 object or homogenenous
% transform matrix (4x4), and N is the number of robot joints. OPTIONS is
% an optional list of name/value pairs than can be passed to fmincon.
%
% Q = robot.ikunc(T, Q0, OPTIONS) as above but specify the
% initial joint coordinates Q0 used for the minimisation.
%
% [Q,ERR] = robot.ikcon(T, ...) as above but also returns ERR which is the
% scalar final value of the objective function.
%
% [Q,ERR,EXITFLAG] = robot.ikcon(T, ...) as above but also returns the
% status EXITFLAG from fmincon.  
%
% [Q,ERR,EXITFLAG,OUTPUT] = robot.ikcon(T, ...) as above but also returns
% a structure with information such as total number of iterations, and final 
% objective function value. See the documentation of fmincon for a complete list.
%
% Trajectory operation::
%
% In all cases if T is a vector of SE3 objects (1xM) or a homogeneous transform
% sequence (4x4xM) then returns the joint coordinates corresponding to
% each of the transforms in the sequence.  Q is MxN where N is the number
% of robot joints. The initial estimate of Q for each time step is taken as
% the solution from the previous time step.
%
% ERR and EXITFLAG are also Mx1 and indicate the results of optimisation
% for the corresponding trajectory step.
%
% Notes::
% - Requires fmincon from the MATLAB Optimization Toolbox.
% - Joint limits are considered in this solution.
% - Can be used for robots with arbitrary degrees of freedom.
% - In the case of multiple feasible solutions, the solution returned
%   depends on the initial choice of Q0.
% - Works by minimizing the error between the forward kinematics of the
%   joint angle solution and the end-effector frame as an optimisation.
%   The objective function (error) is described as:
%           sumsqr( (inv(T)*robot.fkine(q) - eye(4)) * omega )
%   Where omega is some gain matrix, currently not modifiable.
%
% Author::
% Bryan Moutrie
%
% See also SerialLink.ikunc, fmincon, SerialLink.ikine, SerialLink.fkine.

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


function [qstar, error_, exitflag_, out_] = ikcon(robot, T, varargin)
    
    % check if Optimization Toolbox exists, we need it
    assert( exist('fmincon', 'file')>0, 'rtb:ikcon:nosupport', 'Optimization Toolbox required');

    if isa(T, 'SE3')
        T = T.T;
    end
    
    % create output variables
    T_sz = size(T, 3);
    qstar = zeros(T_sz, robot.n);
    error = zeros(T_sz, 1);
    exitflag = zeros(T_sz, 1);
    
    problem.x0 = zeros(1, robot.n);
    problem.options = optimoptions('fmincon', ...
        'Algorithm', 'active-set', ...
        'Display', 'off'); % default options for ikcon
    
    if nargin > 2
        % check if there is a q0 passed
        if isnumeric(varargin{1}) && length(varargin{1}) == robot.n
            problem.x0 = varargin{1};
            varargin = varargin(2:end);
        end
    end
    if ~isempty(varargin)
        % if given, add optional argument to the list of optimiser options
        problem.options = optimoptions(problem.options, varargin{:});
    end
    
    % set the joint limit bounds
    problem.lb = robot.qlim(:,1);
    problem.ub = robot.qlim(:,2);
    problem.solver = 'fmincon';
    
    reach = sum(abs([robot.a, robot.d]));
    omega = diag([1 1 1 3/reach]);
    
    for t = 1: T_sz
        problem.objective = ...
            @(x) sumsqr(((T(:,:,t) \ robot.fkine(x).T) - eye(4)) * omega);
        
        [q_t, err_t, ef_t, out] = fmincon(problem);
        
        qstar(t,:) = q_t;
        error(t) = err_t;
        exitflag(t) = ef_t;
        
        problem.x0 = q_t;
    end
    
    if nargout > 1
        error_ = error;
    end
    if nargout > 2
        exitflag_ = exitflag
    end
    if nargout > 3
        out_ = out;
    end
    
end

function s = sumsqr(A)
    s = sum(A(:).^2);
end

