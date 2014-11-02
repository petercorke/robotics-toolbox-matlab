%COLLISIONS Conduct collision checking for SerialLink objects
%
% Uses the point data stored in the points property of SerialLink
% objects to conduct point-primitive collision checking with objects of
% the CollisionModel class. The function does not currently check the
% base of the SerialLink object!
%
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
%
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
%
% Syntax:
%  (1) c = robot.collisions(q, cmdl)
%  (2) c = robot.collisions(q, cmdl, dyn_cmdl, dyn_T)
%  (3) c = robot.collisions(q, cmdl, dyn_cmdl)
%  (4) c = robot.collisions(q, [], ...)
%
%  (2) is as per (1) but also checks CollisionModel dyn_cmdl, whose 
%       primitives move as specified by the frames in dyn_T
%  (3) is as per (2) but assumes dyn_cmdl is the robot's tool, i.e. the
%       the robot's end-effector frame (including tool transform) is
%       used in lieu of dyn_T
%  (4) is as per (2) or (3) but with no static objects
%
% Outputs:
%  c : mx1 vector, where m is the rows of q. Elements are false if no
%       collision exists or true if a collision exists
%
% Inputs:
%  q        : mxn matrix of m joint configurations of the robot, which
%              has n links
%  cmdl     : CollisionModel object, which is static
%  dyn_cmdl : CollisionModel object which move according to dyn_T
%  dyn_T    : Transforms applied to the primitives before the transform
%              of their own property being applied. The size of dyn_T
%              should be 4x4xmxp, where p is the number of primitives
%              in dyn_cmdl. The ith shape in dyn_cmdl corresponds to the
%              ith value of the 4th dimension. Use cat to create dyn_T.
%
% See also cat CollisionModel SerialLink

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

function c = collisions(robot, q, cmdl, dyn_cmdl, dyn_T)

% VERSION WITH BASE CHECKING
% pts = robot.points;
pts = robot.points(end-robot.n+1:end);
for i = length(pts): -1: 1
    numPts(i) = size(pts{i},1);
    pts{i}(:,4) = 1;
    pts{i} = pts{i}';
end

if isempty(cmdl), checkfuns = []; else checkfuns = cmdl.checkFuns; end
if nargin > 3
    dyn_checkfuns = dyn_cmdl.checkFuns;
    if nargin == 4 || isempty(dyn_T)
        tool = robot.tool;
        dyn_T = [];
    end
else
    dyn_checkfuns = [];
end

% VERSION WITH BASE CHECKING
% base = length(pts) - robot.n;
% switch base
%     case 0
%         % No base
%     case 1
%         T = robot.base;
%         trPts = T * pts{1};
%         points = trPts (1:3,:)';
%         if any(checkfuns{1}(points(:,1),points(:,2),points(:,3)))
%             C(:) = 1;
%             display('Base is colliding');
%             return;
%         end
%     otherwise
%         error('robot has missing or extra points');
% end

poses = size(q,1);
c = false(poses, 1);

nan = any(isnan(q),2);
c(nan) = true;
notnan = find(~nan)';

T0 = robot.base;

for p = notnan
    T = T0;
    prevPts = 0;
    for i = 1: robot.n;
        T = T * robot.links(i).A(q(p,i));
        if numPts(i) % Allows some links to be STLless
            %       VERSION WITH BASE CHECKING
            %       nextPts = prevPts+numPts(i+base);
            %       trPts(:,prevPts+1:nextPts) = T * pts{i+base};
            nextPts = prevPts+numPts(i);
            trPts(:,prevPts+1:nextPts) = T * pts{i};
            prevPts = nextPts;
        end
    end
    
    % Does same thing as cmdl.collision(trPoints(1:3,:)'), but
    % Does not have to access object every time - quicker
    for i = 1: length(checkfuns)
        if any(checkfuns{i}(trPts(1,:), trPts(2,:), trPts(3,:)))
            c(p) = true;
            break;
        end
    end
    
    % Then check the dynamic collision models, if any
    for i = 1: length(dyn_checkfuns)
        if isempty(dyn_T)
            dyn_trPts = T*tool \ trPts;
        else
            dyn_trPts = dyn_T(:,:,p,i) \ trPts;
        end
        if any(dyn_checkfuns{i}(dyn_trPts(1,:), dyn_trPts(2,:), ...
                dyn_trPts(3,:)))
            c(p) = true;
            break;
        end
    end
    
end
end