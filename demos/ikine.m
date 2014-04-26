
% Copyright (C) 1993-2014, by Peter I. Corke
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

% DO VARIOUS ROBOTS: 2 LINK, HYPER2D, HYPER3D

% Inverse kinematics is the problem of finding the robot joint coordinates,
% given a homogeneous transform representing the last link of the manipulator.
% It is very useful when the path is planned in Cartesian space, for instance 
% a straight line path as shown in the trajectory demonstration.
%
% First generate the transform corresponding to a particular joint coordinate,

mdl_puma560
q = [0 -pi/4 -pi/4 0 pi/8 0]
T = p560.fkine(q)

% Now the inverse kinematic procedure for any specific robot can be derived 
% symbolically and in general an efficient closed-form solution can be 
% obtained.  However we are given only a generalized description of the 
% manipulator in terms of kinematic parameters so an iterative solution will 
% be used. The procedure is slow, and the choice of starting value affects 
% search time and the solution found, since in general a manipulator may 
% have several poses which result in the same transform for the last
% link. The starting point for the first point may be specified, or else it
% defaults to zero (which is not a particularly good choice in this case)

qi = p560.ikine(T);

% and in fact it does not converge

qi

% We can help the solution along by using the 'pinv' option

qi = p560.ikine(T, 'pinv');
% and the result
qi
% is the same as the original set of joint angles
q

% However in general this will not be the case, there are multiple
% solutions, and the solution that is found depends on the initial
% choice of angles.
%
% A more efficient approach is to use an analytic solution and the toolbox 
% supports the common case of a 6-axis robot arm with a spherical wrist

qi = p560.ikine6s(T)
% which is different to the original joint angles, but as expected

p560.fkine(qi)

% it does give the same end-effector pose.

% The analytic solution allows the specific solution to be specified
% using a character string and to get the same set of joint angles

p560.ikine6s(T, 'rdf')
% where we have specified that the robot is in a right-handed configuration
% (r), with its elbow down (d), and the wrist flipped (f).

% A solution is not always possible, for instance if the specified 
% transform describes a point out of reach of the manipulator.  As 
% mentioned above the solutions are not necessarily unique, and there 
% are singularities at which the manipulator loses degrees of freedom 
% and joint coordinates become linearly dependent.

% Inverse kinematics may also be computed for a trajectory.
% If we take a Cartesian straight line path between two poses in 50 steps

T1 = transl(0.6, -0.5, 0.0) % define the start point
T2 = transl(0.4, 0.5, 0.2)	% and destination
T = ctraj(T1, T2, 50); 	% compute a Cartesian path

% now solve the inverse kinematics

q = p560.ikine6s(T); 
about q
% which has one row per time step and one column per joint angle

% Let's examine the joint space trajectory that results in straightline 
% Cartesian motion

subplot(3,1,1); plot(q(:,1)); xlabel('Time (s)'); ylabel('Joint 1 (rad)');
subplot(3,1,2); plot(q(:,2)); xlabel('Time (s)'); ylabel('Joint 2 (rad)');
subplot(3,1,3); plot(q(:,3)); xlabel('Time (s)'); ylabel('Joint 3 (rad)');

% This joint space trajectory can now be animated
clf
p560.plot(q)
