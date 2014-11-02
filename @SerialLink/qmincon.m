%QMINCON Resolve redundancy in robots by avoiding joint limits
%
% A popular way to resolve redundant robots is to keep joints away from
% their mechanical limits to allow freer motion. This function will do
% that process. Requires fmincon from the optimization toolbox.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) [qstar, error, exitflag] = robot.qmincon(q)
%
% Outputs:
%  qstar    : Optimised joint angles, mxrobot.n where m = size(q,2)
%  error    : The error measurement (value of objective function)
%  exitflag : The exitflag direct from fmincon
%
% Inputs:
%  q : Joint configuration(s), may be an mxrobot.n matrix of m poses
%
% See also fmincon ikcon ikunc

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

function [qstar, error, exitflag] = qmincon(robot, q)

M = size(q,1);
n = robot.n;

qstar = zeros(M,n);
error = zeros(M,1);
exitflag = zeros(M,1);

opt = optimoptions('fmincon', ...
    'Algorithm', 'active-set', ...
    'Display', 'off');

lb = robot.qlim(:,1);
ub = robot.qlim(:,2);

x_m = 0; % Little trick for setting x0 in first iteration of loop

for m = 1:M
    q_m = q(m,:);
    
    J = robot.jacobn(q(m,:));
    N = null(J);
    
    if isempty(N)
        error(pHRIWARE('error', 'Robot is not redundant'));
    end
    
    f = @(x) sumsqr((2*(N*x + q_m') - ub - lb)./(ub-lb));
    
    x0 = zeros(size(N,2), 1) + x_m;
    
    A = [N; -N];
    b = [ub-q_m'; q_m'-lb];
    
    [x_m, err_m, ef_m] = fmincon(f,x0,A,b,[],[],[],[],[],opt);
    
    qstar(m,:) = q(m,:) + (N*x_m)';
    error(m) = err_m;
    exitflag(m) = ef_m;
end

end

function s = sumsqr(A)
    s = sum(A.^2);
end
