%SerialLink.QMINCON Use redundancy to avoid joint limits
%
% QS = R.qmincon(Q) exploits null space motion and returns a set of joint
% angles QS (1xN) that result in the same end-effector pose but are away
% from the joint coordinate limits.  N is the number of robot joints.
%
% [Q,ERR] = R.qmincon(Q) as above but also returns ERR which is the
% scalar final value of the objective function.
%
% [Q,ERR,EXITFLAG] = R.qmincon(Q) as above but also returns the
% status EXITFLAG from fmincon.
%
% Trajectory operation::
%
% In all cases if Q is MxN it is taken as a pose sequence and R.qmincon()
% returns the adjusted joint coordinates (MxN) corresponding to each of the
% poses in the sequence.
%
% ERR and EXITFLAG are also Mx1 and indicate the results of optimisation
% for the corresponding trajectory step.
%
% Notes::
% - Requires fmincon from the MATLAB Optimization Toolbox.
% - Robot must be redundant.
%
% Author::
% Bryan Moutrie
%
% See also SerialLink.ikcon, SerialLink.ikunc, SerialLink.jacob0.

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

function [qstar, error, exitflag] = qmincon(robot, q)
    
    % check if Optimization Toolbox exists, we need it
    assert( exist('fmincon')>0, 'rtb:qmincon:nosupport', 'Optimization Toolbox required');
    assert( robot.n > 6, 'rtb:qmincon:badarg', 'pHRIWARE:Robot is not redundant');

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
        
        J = robot.jacobe(q(m,:));
        N = null(J);
        
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
