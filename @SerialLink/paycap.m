%PAYCAP Compute the static payload capacity of a SerialLink object
%
% Find the maximum magnitude of a wrench applied at the end-effector
% for a given pose. The wrench may be referenced in the world frame or
% end-effector frame. How loads are calculated vary to minimise time.
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) [wM, j] = robot.paycap(q, w, f, tauR)
%
% Outputs:
%  wM : The maximum permissible magnitude of the wrench
%  j  : The joint which hits its limit at wM
%
% Inputs:
% q    : Joint co-ordinates of the robot, may be a matrix where each 
%        row is a configuration, in which case wM and j are vectors
% w    : Wrench vector (column). The magnitude of w is ignored.
% f    : Reference frame, '0' for world frame or 'n' for end-effector
% tauR : 2xn matrix of maximum/minimum joint forces/torques. The first
%         row is maximum, second minimum. The number of joints is n.
%
% See also grav, pay

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

function [wM, j] = paycap(robot, q, w, f, tauR)

if robot.fast
    tauB = robot.gravload(q);
    tauP = robot.frne(q, 0*q, 0*q, [0; 0; 0], unit(w));
elseif f == '0'
    [tauB, J] = grav(robot, q);
    tauP = pay(unit(w), J);
elseif f == 'n'
    tauB = grav(robot, q);
    tauP = pay(robot, unit(w), q, 'n'); 
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

