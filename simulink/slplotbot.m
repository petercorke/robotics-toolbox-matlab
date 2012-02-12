%SLPLOTBOT	S-function for robot animation
%
% This is the S-function for animating the robot.  It assumes input
% data u to be the joint angles q.
%
% Implemented as an S-function so as to update display at the end of
% each Simulink major integration step.

function [sys,x0,str,ts] = splotbot(t,x,u,flag, robot)
	switch flag,

	case 0
		% initialize the robot graphics
		[sys,x0,str,ts] = mdlInitializeSizes;	% Init
		plot(robot, zeros(1, robot.n))

	case 2
		% come here on update
		if ~isempty(u),
			plot(robot, u');
			drawnow
		end
		ret = [];
	case {1, 4, 9}
		ret = [];
	end
%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes
 
%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;
 
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 0;
sizes.NumInputs      = -1;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
 
sys = simsizes(sizes);
 
%
% initialize the initial conditions
%
x0  = [];
 
%
% str is always an empty matrix
%
str = [];
 
%
% initialize the array of sample times
%
ts  = [0 0];
 
% end mdlInitializeSizes
