%DELTA2TR Convert differential motion  to a homogeneous transform
%
% T = DELTA2TR(D) is a homogeneous transform representing differential 
% translation and rotation. The vector D=(dx, dy, dz, dRx, dRy, dRz)
% represents an infinitessimal motion, and is an approximation to the spatial 
% velocity multiplied by time.
%
% See also TR2DELTA.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function delta = delta2tr(d)
    d = d(:);
    delta = eye(4,4) + [skew(d(4:6)) d(1:3); 0 0 0 0];
