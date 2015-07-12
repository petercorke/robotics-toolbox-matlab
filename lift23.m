%SE3 Lift SE(2) transform to SE(3)
%
% T3 = SE3(T2) returns a homogeneous transform (4x4) that represents
% the same X,Y translation and Z rotation as does T2 (3x3).
%
% See also SE2, TRANSL, ROTX.


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
function T = se3(x)

    if all(size(x) == [3 3])
        T = [x(1:2,1:2) [0 0]' x(1:2,3); 0 0 1 0; 0 0 0 1];
     end
