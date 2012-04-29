%VEX Convert skew-symmetric matrix to vector
%
% V = VEX(S) is the vector (3x1) which has the skew-symmetric matrix S (3x3)
%
%           | 0   -vz  vy|
%           | vz   0  -vx|
%           |-vy   vx  0 |
%
% Notes::
% - This is the inverse of the function SKEW().
% - No checking is done to ensure that the matrix is actually skew-symmetric.
% - The function takes the mean of the two elements that correspond to each unique
%   element of the matrix, ie. vx = 0.5*(S(3,2)-S(2,3))
%
% See also SKEW.


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

function v = vex(S)
    if isrot(S) || ishomog(S)
        v = 0.5*[S(3,2)-S(2,3); S(1,3)-S(3,1); S(2,1)-S(1,2)];
    else
        error('argument must be a 3x3 matrix');
    end
