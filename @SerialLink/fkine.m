%SerialLink.fkine  Forward kinematics
%
% T = R.fkine(Q) is the pose (4x4) of the robot end-effector as a homogeneous
% transformation for the joint configuration Q (1xN).
%
% If Q is a matrix (KxN) the rows are interpretted as the generalized
% joint coordinates for a sequence of points along a trajectory.  Q(i,j) is
% the j'th joint parameter for the i'th trajectory point.  In this case
% T is a 3d matrix (4x4xK) where the last subscript is the index along the path.
%
% Note::
% - The robot's base or tool transform, if present, are incorporated into the
%   result.
% - Joint offsets, if defined, are added to Q before the forward kinematics are
%   computed.
%
% See also SerialLink.ikine, SerialLink.ikine6s.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

% Modifications by Joern Malzahn to support CodeGenerator functionality

function [t allt] = fkine(robot, q)
%
% evaluate fkine for each point on a trajectory of
% theta_i or q_i data
%

n = robot.n;

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
