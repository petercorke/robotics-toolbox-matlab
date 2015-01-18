
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
% if you have a gaming joystick connected to your computer you use the 
% sticks to control the position and orientation of a coordinate frame.

T = eye(4,4);
h = trplot(T, 'axis', [-5 5 -5 5 -5 5]);

while true
    T = joy2tr(T, 'tool');
    trprint(T, 'fmt', '%.1f')
    trplot(h, T);
end
