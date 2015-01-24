%SerialLink.PAYCAP Static payload capacity of a robot
%
% [WMAX,J] = R.paycap(Q, W, F, TLIM) returns the maximum permissible
% payload wrench WMAX (1x6) applied at the end-effector, and the index of
% the joint J which hits its force/torque limit at that wrench.  Q (1xN) is
% the manipulator pose, W the payload wrench (1x6), F the wrench reference
% frame (either '0' or 'n') and TLIM (2xN) is a matrix of joint
% forces/torques (first row is maximum, second row minimum).
%
% Trajectory operation::
%
% In the case Q is MxN then WMAX is Mx6 and J is Mx1 where the rows are the
% results at the pose given by corresponding row of Q.
%
% Notes::
% - Wrench vector and Jacobian must be from the same reference frame
% - Tool transforms are taken into consideration for F = 'n'.
%
% Author::
% Bryan Moutrie
%
% See also SerialLink.pay, SerialLink.gravjac, SerialLink.gravload.

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

function [wM, j] = paycap(robot, q, w, f, tauR)
    
    if robot.fast
        tauB = robot.gravload(q);
        tauP = robot.rne(q, zeros(size(q)), zeros(size(q)), [0; 0; 0], unit(w));
    elseif f == '0'
        [tauB, J] = gravjac(robot, q);
        tauP = robot.pay(unit(w), J);
    elseif f == 'n'
        tauB = gravjac(robot, q);
        tauP = robot.pay(unit(w), q, 'n');
    end
    
    M = tauP > 0;
    m = ~M;
    
    TAUm = ones(size(tauB));
    TAUM = ones(size(tauB));
    for c = 1:robot.n
        TAUM(:,c) = tauR(1,c);
        TAUm(:,c) = tauR(2,c);
    end
    
    WM = zeros(size(tauB));
    WM(M) = (TAUM(M) - tauB(M)) ./ tauP(M);
    WM(m) = (TAUm(m) - tauB(m)) ./ tauP(m);
    
    WM(isinf(WM)) = Inf; % Makes -Inf values Inf
    
    [wM, j] = min(WM,[],2);
    
end

function u = unit(v)
    u = v/norm(v);
end

