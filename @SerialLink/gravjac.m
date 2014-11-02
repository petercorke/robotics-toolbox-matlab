% GRAV Quick non-mex gravload and jacob0 for SerialLink objects
% 
% Calculates the joint loads due to gravity for SerialLink objects. The
% world frame Jacobian can also be returned. The gravity vector is
% defined by the SerialLink property if not explicitly given.
% 
% Copyright (C) Bryan Moutrie, 2013-2014
% Licensed under the GNU Lesser General Public License
% see full file for full statement
% 
% This file requires file(s) from The Robotics Toolbox for MATLAB (RTB)
% by Peter Corke (www.petercorke.com), see file for statement
% 
% Syntax:
%  (1) tauB = robot.grav(q)
%  (2) [tauB, J] = robot.grav(q)
%  (3) ... = robot.grav(q, grav)
% 
%  (2) as per (1) but also returns world-frame Jacobian at little time
%  (3) as per others but with explicit gravity vector
% 
% Outputs:
%  tauB : Generalised joint force/torques
%  J    : Jacobian in world frame
% 
% Inputs:
%  robot : SerialLink object (sdh or mdh, R and P supported)
%  q     : Joint row vector, or trajectory matrix of joint row vectors
%  grav  : Gravity _reaction_ vector (as RTB default is [0 0 +9.81]')
%
% See also pay, SerialLink, SerialLink.gravload, jacob0

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

function [tauB, J] = grav(robot, q, grav)

n = robot.n;
revolute = [robot.links(:).isrevolute];
if ~robot.mdh
    baseAxis = robot.base(1:3,3);
    baseOrigin = robot.base(1:3,4);
end

poses = size(q, 1);
tauB = zeros(poses, n);
if nargout == 2, J = zeros(6, robot.n, poses); end

% Forces
force = zeros(3,n);
if nargin < 3, grav = robot.gravity; end
for joint = 1: n
    force(:,joint) = robot.links(joint).m * grav;
end

% Centre of masses (local frames)
r = zeros(4,n);
for joint = 1: n
    r(:,joint) = [robot.links(joint).r'; 1];
end
com_arr = zeros(3, n);

for pose = 1: poses
    
    [Te, T] = robot.fkine(q(pose,:));
    
    jointOrigins = squeeze(T(1:3,4,:));
    jointAxes = squeeze(T(1:3,3,:));
    
    if ~robot.mdh
        jointOrigins = [baseOrigin, jointOrigins(:,1:end-1)];
        jointAxes = [baseAxis, jointAxes(:,1:end-1)];  
    end
    
    % Backwards recursion
    for joint = n: -1: 1
        
        com = T(:,:,joint) * r(:,joint); % C.o.M. in world frame, homog
        com_arr(:,joint) = com(1:3); % Add it to the distal others
        
        t = 0;
        for link = joint: n % for all links distal to it
            if revolute(joint)
                d = com_arr(:,link) - jointOrigins(:,joint);
                t = t + cross3(d, force(:,link));
            % Though r x F would give the applied torque and not the
            % reaction torque, the gravity vector is nominally in the
            % positive z direction, not negative, hence the force is
            % the reaction force
            else
                t = t + force(:,link); %force on prismatic joint
            end
        end
        
        tauB(pose,joint) = t' * jointAxes(:,joint);
    end
    
    if nargout == 2
        J(:,:,pose) = makeJ(jointOrigins,jointAxes,Te(1:3,4),revolute);
    end

end


end

function J = makeJ(O,A,e,r)
J(4:6,:) = A;
for j = 1:length(r)
    if r(j)
        J(1:3,j) = cross3(A(:,j),e-O(:,j));
    else
        J(:,j) = J([4 5 6 1 2 3],j); %J(1:3,:) = 0;
    end
end
end

function c = cross3(a,b)
c(3,1) = a(1)*b(2) - a(2)*b(1);
c(1,1) = a(2)*b(3) - a(3)*b(2);
c(2,1) = a(3)*b(1) - a(1)*b(3);
end
