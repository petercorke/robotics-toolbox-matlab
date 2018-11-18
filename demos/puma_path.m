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

%%begin

We animate a Puma 560 robot moving between 4 points on a virtual tabletop.

% declare the robot and set tool length
mdl_puma560
p560.tool = SE3(0,0,0.2)

s = 0.2;   % square side length
% define the square and orientation for each segment, the gripper reorients
% at each corner

%       x  y  z  theta
p = [   s  s  0  pi/2   % start
       -s  s  0  pi/2   % side 1
       -s  s  0  0      % rotate
       -s -s  0  0      % side 2
       -s -s  0  pi/2   % rotate
        s -s  0  pi/2   % side 3
        s -s  0  0      % rotate
        s  s  0  0      % side 4
    ];

t1 = 5; % time for a side
t2 = 1; % time for a turn

% generate a trajectory, a matrix with each row being (x,y,z,theta)
pt = mstraj(p, [], [t1 t2 t1 t2 t1 t2 t1], [], 0.1, 0.4);

% translation for each point on the trajectory
X = SE3([0.5 0 -s]);

% each point on the trajectory, translate it, set the z-axis downward and
% rotate about the gripper z-axis
Tt = SE3(pt(:,1:3)) * X * SE3.Ry(pi) * SE3.Rz(pt(:,4));

% now do inverse kinematics with the elbow up
qt = p560.ikine6s(Tt, 'ru');


% get the positions of corners, the odd waypoints
ps = X * p(1:2:end,1:3)'

% place a small sphere there
clf
plot_sphere(ps, 0.03, 'y');

% animate a 3D Puma model with a thick trailing line
p560.plot3d(qt, 'trail', {'r', 'LineWidth', 8}, 'nowrist', 'view', [138 8]);