%SerialLink.JACOBE Jacobian in end-effector frame
%
% JE = R.jacobe(Q, options) is the Jacobian matrix (6xN) for the robot in
% pose Q, and N is the number of robot joints. The manipulator Jacobian
% matrix maps joint velocity to end-effector spatial velocity V = JE*QD in
% the end-effector frame.
%
% Options::
% 'trans'   Return translational submatrix of Jacobian
% 'rot'     Return rotational submatrix of Jacobian 
%
% Notes::
% - Was joacobn() is earlier version of the Toolbox.
% - This Jacobian accounts for a tool transform if one is set.
% - This Jacobian is often referred to as the geometric Jacobian.
% - Prior to release 10 this function was named jacobn.
%
% References::
%  - Differential Kinematic Control Equations for Simple Manipulators,
%    Paul, Shimano, Mayer,
%    IEEE SMC 11(6) 1981,
%    pp. 456-460
%
% See also SerialLink.jacob0, jsingu, delta2tr, tr2delta.



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

function J = jacobe(robot, q, varargin)

    opt.trans = false;
    opt.rot = false;
    opt.deg = false;
    opt = tb_optparse(opt, varargin);
    if opt.deg
        % in degrees mode, scale the columns corresponding to revolute axes
        q = robot.toradians(q);
    end
    
    n = robot.n;
    L = robot.links;        % get the links
    
   
    J = zeros(6, robot.n);
    if isa(q, 'sym')
        J = sym(J);
    end
    
    U = robot.tool;
    for j=n:-1:1
        if robot.mdh == 0
            % standard DH convention
            U = L(j).A(q(j)) * U;
        end
        
        UT = U.T;

        if L(j).isrevolute
            % revolute axis
            d = [   -UT(1,1)*UT(2,4)+UT(2,1)*UT(1,4)
                -UT(1,2)*UT(2,4)+UT(2,2)*UT(1,4)
                -UT(1,3)*UT(2,4)+UT(2,3)*UT(1,4)];
            delta = UT(3,1:3)';  % nz oz az
        else
            % prismatic axis
            d = UT(3,1:3)';      % nz oz az
            delta = zeros(3,1); %  0  0  0
        end
        J(:,j) = [d; delta];

        if robot.mdh ~= 0
            % modified DH convention
            U = L(j).A(q(j)) * U;
        end
    end
    
    if opt.trans
        J = J(1:3,:);
    elseif opt.rot
        J = J(4:6,:);
    end
    
    if isa(J, 'sym')
        J = simplify(J);
    end
