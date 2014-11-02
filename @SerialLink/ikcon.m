%IKCON Compute inverse kinematics considering joint limits
%
% Computes the inverse kinematics for an arbitrary SerialLink
% manipulator. Requires fmincon from the optimization toolbox. ikcon
% works by minimizing the error between the forward kinematics of the
% joint angle solution and the end-effector frame as an optimisation.
% The objective function (error) is described as:
%           sumsqr( (inv(T)*robot.fkine(q) - eye(4)) * omega )
% Where omega is some gain matrix, currently not modifiable.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) [qstar, error, exitflag] = robot.ikcon(T)
%  (2) [qstar, error, exitflag] = robot.ikcon(T, q0)
%  (3) [qstar, error, exitflag] = robot.ikcon(T, problem)
%
%  (2) is as per (1) but specifies a starting pose instead of zeroes
%  (3) instead uses an explicit problem structure for fmincon to use
%
% Outputs:
%  qstar    : Optimised joint angles, mxrobot.n where m = size(T,3)
%  error    : The error measurement (value of objective function)
%  exitflag : The exitflag direct from fmincon
%
% Inputs:
%  T       : End-effector frame. May be a 4x4xm sequence of frames, in
%             which case for 2nd to mth frame, the previous qstar value
%             is used for q0
%  q0      : Starting point for optimisation algorithm (first guess)
%  problem : A structure containing problem data as descibed in fmincon
%             documentation. The fields lb, ub, solver and objective 
%             need not be specified, as these are assigned within ikcon
%
% See also fmincon ikunc qmincon SerialLink.ikine

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
%
% RTB LIBRARY:
%
% Copyright (C) 1993-2014, by Peter I. Corke
% http://www.petercorke.com
% Released under the GNU Lesser General Public license

function [qstar, error, exitflag] = ikcon(robot, T, varargin)

T_sz = size(T, 3);
qstar = zeros(T_sz, robot.n);
error = zeros(T_sz, 1);
exitflag = zeros(T_sz, 1);

problem.x0 = zeros(1, robot.n);
opt = optimoptions('fmincon', ...
    'Algorithm', 'active-set', ...
    'Display', 'off');
if ~isempty(varargin) && isstruct(varargin{1})
    problem = varargin{1};
    problem.options = optimset(opt, problem.options);
else
    if ~isempty(varargin), problem.x0 = varargin{1}; end
    problem.options = opt;
end
problem.lb = robot.qlim(:,1);
problem.ub = robot.qlim(:,2);
problem.solver = 'fmincon';

reach = sum([robot.a, robot.d]);
omega = diag([1 1 1 3/reach]);

for t = 1: T_sz
    problem.objective = ...
        @(x) sumsqr(((T(:,:,t) \ robot.fkine(x)) - eye(4)) * omega);
    
    [q_t, err_t, ef_t] = fmincon(problem);
    
    qstar(t,:) = q_t;
    error(t) = err_t;
    exitflag(t) = ef_t;
    
    problem.x0 = q_t;
end

end

function s = sumsqr(A)
    s = sum(A(:).^2);
end

