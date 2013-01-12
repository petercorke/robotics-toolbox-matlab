%%begin

% We will work with a model of the Puma 560 robot
mdl_puma560

% to make the numerical simulation work faster we will remove the non-linear
% Coloumb friction that exists in the model

p560 = p560.nofriction()

% A simple Simulink model that connects a robot dynamics block
% to a robot plot block

sl_ztorque

% The robot has zero applied torque at each joint, so when we run the simulation it
% will collapse under its own weight

sim('sl_ztorque')
