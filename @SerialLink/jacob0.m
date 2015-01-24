%SerialLink.JACOB0 Jacobian in world coordinates
%
% J0 = R.jacob0(Q, OPTIONS) is the Jacobian matrix (6xN) for the robot in
% pose Q (1xN), and N is the number of robot joints.  The manipulator
% Jacobian matrix maps joint velocity to end-effector spatial velocity V =
% J0*QD expressed in the world-coordinate frame.
%
% Options::
% 'rpy'     Compute analytical Jacobian with rotation rate in terms of 
%           roll-pitch-yaw angles
% 'eul'     Compute analytical Jacobian with rotation rates in terms of 
%           Euler angles
% 'trans'   Return translational submatrix of Jacobian
% 'rot'     Return rotational submatrix of Jacobian 
%
% Note::
% - The Jacobian is computed in the end-effector frame and transformed to
%   the world frame.
% - The default Jacobian returned is often referred to as the geometric 
%   Jacobian, as opposed to the analytical Jacobian.
%
% See also SerialLink.jacobn, jsingu, deltatr, tr2delta, jsingu.


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

function J0 = jacob0(robot, q, varargin)

    opt.rpy = false;
    opt.eul = false;
    opt.trans = false;
    opt.rot = false;
    
    opt = tb_optparse(opt, varargin);
    
	%
	%   dX_tn = Jn dq
	%
	Jn = jacobn(robot, q);	% Jacobian from joint to wrist space

	%
	%  convert to Jacobian in base coordinates
	%
	Tn = fkine(robot, q);	% end-effector transformation
	R = t2r(Tn);
	J0 = [R zeros(3,3); zeros(3,3) R] * Jn;

    if opt.rpy
        rpy = tr2rpy( fkine(robot, q) );
        B = rpy2jac(rpy);
        if rcond(B) < eps
            error('Representational singularity');
        end
        J0 = blkdiag( eye(3,3), inv(B) ) * J0;
    elseif opt.eul
        eul = tr2eul( fkine(robot, q) );
        B = eul2jac(eul);
        if rcond(B) < eps
            error('Representational singularity');
        end
        J0 = blkdiag( eye(3,3), inv(B) ) * J0;
    end
    
    if opt.trans
        J0 = J0(1:3,:);
    elseif opt.rot
        J0 = J0(4:6,:);
    end
