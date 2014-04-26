
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
% In the field of robotics there are many possible ways of representing 
% positions and orientations, but the homogeneous transformation is well 
% matched to MATLABs powerful tools for matrix manipulation.
%
% Homogeneous transformations describe the relationships between Cartesian 
% coordinate frames in terms of translation and orientation.  

%  A pure translation of 0.5m in the X direction is represented by

transl(0.5, 0.0, 0.0)

% a rotation of 90degrees about the Y axis by

troty(pi/2)

% and a rotation of -90degrees about the Z axis by

trotz(-pi/2)

%  these may be concatenated by multiplication

t = transl(0.5, 0.0, 0.0) * troty(pi/2) * trotz(-pi/2)

%
% If this transformation represented the origin of a new coordinate frame with respect
% to the world frame origin (0, 0, 0), that new origin would be given by

t * [0 0 0 1]'

% the orientation of the new coordinate frame may be expressed in terms of
% Euler angles

tr2eul(t)

% or roll/pitch/yaw angles

tr2rpy(t)

% It is important to note that tranform multiplication is in general not 
% commutative as shown by the following example

trotx(pi/2) * trotz(-pi/8)
trotz(-pi/8) * trotx(pi/2)
