%JOYSTICK Input from joystick
%
% J = JOYSTICK() returns a vector of joystick values in the range -1 to +1.
%
% [J,B] = JOYSTICK() as above but also returns a vector of button values, 
% either 0 (not pressed) or 1 (pressed).
%
% Notes::
% - This is a MEX file that uses SDL (www.libsdl.org) to interface to a standard
%   gaming joystick.
% - The length of the vectors J and B depend on the capabilities of the
%   joystick identified when it is first opened.
%
% See also joy2tr.

% Copyright (C) 1993-2015, by Peter I. Corke
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
%   joystick identified when it is first opened.
