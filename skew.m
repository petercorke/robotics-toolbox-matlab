%SKEW Create skew-symmetric matrix
%
% S = SKEW(V) is a skew-symmetric matrix formed from V (3x1).
%
%           | 0   -vz  vy|
%           | vz   0  -vx|
%           |-vy   vx  0 |
%
% See also VEX.


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

function S = skew(v)
    if isvec(v,3)
        % SO(3) case
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];
    elseif isvec(v,1)
        % SO(2) case
        S = [0 -v; v 0];
    else
        error('argument must be a 1- or 3-vector');
    end
