
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

% The trajectory demonstration has shown how a joint coordinate trajectory
% may be generated

t = [0:.05:2]'; 	% generate a time vector
q = jtraj(qz, qr, t); % generate joint coordinate trajectory

% the overloaded function plot() animates a stick figure robot moving 
% along a trajectory.

p560.plot(q);

% The drawn line segments do not necessarily correspond to robot links, but 
% join the origins of sequential link coordinate frames.
%
% A small right-angle coordinate frame is drawn on the end of the robot to show
% the wrist orientation.
%
% A shadow appears on the ground which helps to give some better idea of the
% 3D object.
%
% This is a 3D plot so using the tools on the figure toolbar you can rotate the
% figure and change your viewpoint.

%--
% We can also place additional robots into a figure.
%
% Let's make a clone of the Puma robot, but change its name and base location

p560_2 = SerialLink(p560, 'name', 'another Puma', 'base', transl(-0.5, 0.5, 0) )
hold on
p560_2.plot(q);

% We can also have multiple views of the same robot

clf
p560.plot(qr);
figure
p560.plot(qr);
view(40,50)

% so now when we animate the robot, it moves in both views
p560.plot(q)

% Sometimes it's useful to be able to manually drive the robot around to
% get an understanding of how it works.

p560.teach()
% Use the sliders to control the robot (in fact both views).  Hit the 
% red X button when you are done.

% you can find the final joint angles of the robot you drove it to
p560.getpos()
