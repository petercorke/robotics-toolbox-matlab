%SLACCEL	S-function for robot acceleration
%
% This is the S-function for computing robot acceleration. It assumes input
% data u to be the vector [q qd tau].
%
% Implemented as an S-function to get around vector sizing problem with
% Simulink 4.

function [sys, x0, str, ts] = slaccel(t, x, u, flag, robot)
	switch flag,

	case 0
		% initialize the robot graphics
		[sys,x0,str,ts] = mdlInitializeSizes(robot);	% Init

	case {3}
		% come here to calculate derivitives
        
        % first check that the torque vector is sensible
        if length(u) ~= (3*robot.n)
            error('RTB:slaccel:badarg', 'Input vector is length %d, should be %d', length(u), 3*robot.n);
        end
        if ~isreal(u)
            error('RTB:slaccel:badarg', 'Input vector is complex, should be real'); 
        end
        
		sys = robot.accel(u(:)');
	case {1, 2, 4, 9}
		sys = [];
	end
%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(robot)
 
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
sizes.NumOutputs     = robot.n;
sizes.NumInputs      = 3*robot.n;
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
