%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   PARAMETERS Returns a data structure containing the parameters of the
%   UNIMATE PUMA 560 robot.
%
%   Author: Arturo Gil. Universidad Miguel Hern�ndez de Elche. 
%   email: arturo.gil@umh.es date:   03/03/2012
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright (C) 2012, by Arturo Gil Aparicio
%
% This file is part of ARTE (A Robotics Toolbox for Education).
% 
% ARTE is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% ARTE is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
function robot = parameters()
%KYNEMATICS
robot.name= 'puma_560';

robot.DH.theta = '[q(1) q(2) q(3) q(4) q(5) q(6)]';
robot.DH.d='[0  0   0.15005  0.4318     0    0.04]';
robot.DH.a='[0 0.4318  0.0203      0       0    0]';
robot.DH.alpha= '[pi/2  0   -pi/2  pi/2   -pi/2 0]';
robot.J=[];
robot.name='Puma 560 robotic arm';

robot.inversekinematic_fn = 'inversekinematic_puma560(robot, T)';


%R: rotational, T: translational
robot.kind=['R' 'R' 'R' 'R' 'R' 'R'];

%number of degrees of freedom
robot.DOF = 6;

%minimum and maximum rotation angle in rad
robot.maxangle =[deg2rad(-160) deg2rad(160); %Axis 1, minimum, maximum
                deg2rad(-110) deg2rad(110); %Axis 2, minimum, maximum
                deg2rad(-135) deg2rad(135); %Axis 3
                deg2rad(-266) deg2rad(266); %Axis 4: Unlimited (400� default)
                deg2rad(-100) deg2rad(100); %Axis 5
                deg2rad(-266) deg2rad(266)]; %Axis 6: Unlimited (800� default)

%maximum absolute speed of each joint rad/s or m/s
robot.velmax = [1
                1
                1
                1
                1
                1];%not available
% end effectors maximum velocity
robot.linear_velmax = 0.5; %m/s, from datasheet
robot.accelmax=robot.velmax/0.1; % 0.1 is here an acceleration time

%base reference system
robot.T0 = eye(4);

%INITIALIZATION OF VARIABLES REQUIRED FOR THE SIMULATION
%position, velocity and acceleration
robot=init_sim_variables(robot);
robot.path = pwd;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GRAPHICS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%read graphics files
robot.graphical.has_graphics=1;
robot.graphical.color = [150 180 130]./255;
%for transparency
robot.graphical.draw_transparent=0;
%draw DH systems
robot.graphical.draw_axes=1;
%DH system length and Font size, standard is 1/10. Select 2/20, 3/30 for
%bigger robots
robot.graphical.axes_scale=1;
%adjust for a default view of the robot
robot.axis=[-1 1 -1 1 -0.66 2];
robot = read_graphics(robot);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%DYNAMIC PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robot.has_dynamics=1;

%consider friction in the computations
robot.dynamics.friction=0;

%link masses (kg)
robot.dynamics.masses=[0 17.4 4.8 0.82 0.34 0.09];

%COM of each link with respect to own reference system
robot.dynamics.r_com=[0       0          0; %(rx, ry, rz) link 1
    -0.3638	 0.006	 0.2275; %(rx, ry, rz) link 2
    -0.0203	-0.0141	 0.070;  %(rx, ry, rz) link 3
    0       0.019       0;%(rx, ry, rz) link 4
    0       0           0;%(rx, ry, rz) link 5
   0         0         -0.008];%(rx, ry, rz) link 6

%Inertia matrices of each link with respect to its D-H reference system.
% Ixx	Iyy	Izz	Ixy	Iyz	Ixz, for each row
robot.dynamics.Inertia=[0      0.35	0   	0	0	0;
    0.13    0.524	0.539	0	0	0;
    0.066	0.086	0.0125	0	0	0;
    1.8e-3	1.3e-3	1.8e-3	0	0	0;
    0.3e-3	0.4e-3	0.3e-3	0	0	0;
    0.15e-3	0.15e-3	0.04e-3	0	0	0];


%Please note that we are simulating the motors as presented in MAXON
%catalog
robot.motors=load_motors([5 5 5 5 5 5]);


%Actuator rotor inertia
%robot.motors.Inertia=[200e-6 200e-6 200e-6 33e-6 33e-6 33e-6];
%Speed reductor at each joint
%robot.motors.G=[-62.6111 107.815 -53.7063 76.0364 71.923 76.686];
%Please note that, for simplicity in control, we consider that the gear
%ratios are all positive
robot.motors.G=[62.6111 107.815 53.7063 76.0364 71.923 76.686];
