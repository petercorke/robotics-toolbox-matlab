%SerialLink.fkine Forward kinematics
%
% T = R.fkine(Q, OPTIONS) is the pose of the robot end-effector as an SE(3)
% homogeneous transformation (4x4) for the joint configuration Q (1xN).
%
% If Q is a matrix (KxN) the rows are interpreted as the generalized joint
% coordinates for a sequence of points along a trajectory.  Q(i,j) is the
% j'th joint parameter for the i'th trajectory point.  In this case T is a
% 3d matrix (4x4xK) where the last subscript is the index along the path.
%
% [T,ALL] = R.fkine(Q) as above but ALL (4x4xN) is the pose of the link
% frames 1 to N, such that ALL(:,:,k) is the pose of link frame k.
%
% Options::
%  'deg'    Assume that revolute joint coordinates are in degrees not
%           radians
%
% Note::
% - The robot's base or tool transform, if present, are incorporated into the
%   result.
% - Joint offsets, if defined, are added to Q before the forward kinematics are
%   computed.
%
% See also SerialLink.ikine, SerialLink.ikine6s.


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

function [t allt] = fkine(robot, q, varargin)
    
%
% evaluate fkine for each point on a trajectory of
% theta_i or q_i data
%

n = robot.n;

opt.deg = false;

opt = tb_optparse(opt, varargin);

if opt.deg
    % in degrees mode, scale the columns corresponding to revolute axes
    k = ~robot.links.isprismatic;
    q(:,k) = q(:,k) * pi/180;
end

if nargout > 1
    allt = zeros(4,4,n);
    if isa(q,'sym')
        allt = sym(allt);
    end
end

L = robot.links;
if numel(q) == n
    t = robot.base;
    for i=1:n
        t = t * L(i).A(q(i));
        
        if nargout > 1
            allt(:,:,i) = t; % intermediate transformations
        end
    end
    t = t * robot.tool;
else
    if numcols(q) ~= n
        error('q must have %d columns', n)
    end
    t = zeros(4,4,0);
    for qv=q',		% for each trajectory point
        tt = robot.base;
        for i=1:n
            tt = tt * L(i).A(qv(i));
        end
        t = cat(3, t, tt * robot.tool);
    end
end

if isa(t, 'sym')
    t = simplify(t);
end

%robot.T = t;
%robot.notify('Moved');
