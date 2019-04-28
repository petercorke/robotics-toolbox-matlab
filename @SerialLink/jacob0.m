%SerialLink.JACOB0 Jacobian in world coordinates
%
% J0 = R.jacob0(Q, OPTIONS) is the Jacobian matrix (6xN) for the robot in
% pose Q (1xN), and N is the number of robot joints.  The manipulator
% Jacobian matrix maps joint velocity to end-effector spatial velocity V =
% J0*QD expressed in the world-coordinate frame.
%
% Options::
% 'rpy'     Compute analytical Jacobian with rotation rate in terms of 
%           XYZ roll-pitch-yaw angles
% 'eul'     Compute analytical Jacobian with rotation rates in terms of 
%           Euler angles
% 'exp'     Compute analytical Jacobian with rotation rates in terms of 
%           exponential coordinates
% 'trans'   Return translational submatrix of Jacobian
% 'rot'     Return rotational submatrix of Jacobian 
%
% Note::
% - End-effector spatial velocity is a vector (6x1): the first 3 elements
%   are translational velocity, the last 3 elements are rotational velocity
%   as angular velocity (default), RPY angle rate or Euler angle rate.
% - This Jacobian accounts for a base and/or tool transform if set.
% - The Jacobian is computed in the end-effector frame and transformed to
%   the world frame.
% - The default Jacobian returned is often referred to as the geometric 
%   Jacobian.
%
% See also SerialLink.jacobe, jsingu, deltatr, tr2delta, jsingu.



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

function J0 = jacob0(robot, q, varargin)

    opt.trans = false;
    opt.rot = false;
    opt.analytic = {[], 'rpy', 'eul', 'exp'};
    opt.deg = false;
    
    opt = tb_optparse(opt, varargin);
        if opt.deg
    % in degrees mode, scale the columns corresponding to revolute axes
    q = robot.toradians(q);
end
    
	%
	%   dX_tn = Jn dq
	%
	Jn = jacobe(robot, q);	% Jacobian from joint to wrist space

	%
	%  convert to Jacobian in base coordinates
	%
	Tn = fkine(robot, q);	% end-effector transformation
    if isa(Tn, 'SE3')
        R = Tn.R;
    else
        R = t2r(Tn);
    end
	J0 = [R zeros(3,3); zeros(3,3) R] * Jn;

    % convert to analytical Jacobian if required
    if ~isempty(opt.analytic)
        switch opt.analytic
            case 'rpy'
                rpy = tr2rpy(Tn);
                A = rpy2jac(rpy, 'xyz');
                if rcond(A) < eps
                    error('Representational singularity');
                end
            case 'eul'
                eul = tr2eul(Tn);
                A = eul2jac(eul);
                if rcond(A) < eps
                    error('Representational singularity');
                end
            case 'exp'
                [theta,v] = trlog( t2r(Tn) );
                A = eye(3,3) - (1-cos(theta))/theta*skew(v) ...
                    + (theta - sin(theta))/theta*skew(v)^2;
        end
        J0 = blkdiag( eye(3,3), inv(A) ) * J0;
    end
    
    % choose translational or rotational subblocks
    if opt.trans
        J0 = J0(1:3,:);
    elseif opt.rot
        J0 = J0(4:6,:);
    end
