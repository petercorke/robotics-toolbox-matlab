
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
