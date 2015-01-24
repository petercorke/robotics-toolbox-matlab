%SerialLink.COLLISIONS Perform collision checking
%
% C = R.collisions(Q, MODEL) is true if the SerialLink object R at
% pose Q (1xN) intersects the solid model MODEL which belongs to the
% class CollisionModel.  The model comprises a number of geometric
% primitives and associate pose.
%
% C = R.collisions(Q, MODEL, DYNMODEL, TDYN) as above but also checks
% dynamic collision model DYNMODEL whose elements are at pose TDYN.
% TDYN is an array of transformation matrices (4x4xP), where
% P = length(DYNMODEL.primitives). The P'th plane of TDYN premultiplies the
% pose of the P'th primitive of DYNMODEL.
%
% C = R.collisions(Q, MODEL, DYNMODEL) as above but assumes TDYN is the
% robot's tool frame.
%
% Trajectory operation::
%
% If Q is MxN it is taken as a pose sequence and C is Mx1 and the collision
% value applies to the pose of the corresponding row of Q. TDYN is 4x4xMxP.
%
% Notes::
% - Requires the pHRIWARE package which defines CollisionModel class.
%   Available from: https://code.google.com/p/phriware/ .
% - The robot is defined by a point cloud, given by its points property.
% - The function does not currently check the base of the SerialLink
%   object.
% - If MODEL is [] then no static objects are assumed.
%
% Author::
% Bryan Moutrie
%
% See also CollisionModel, SerialLink.

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

function c = collisions(robot, q, cmdl, dyn_cmdl, dyn_T)
    
    if ~exist('pHRIWARE')
        error('rtb:collisions:nosupport', 'You need to install pHRIWARE in order to use this functionality');
    end
    
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
