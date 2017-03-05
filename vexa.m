%VEXA Convert augmented skew-symmetric matrix to vector
%
% V = VEXA(S) is the vector which has the corresponding augmented skew-symmetric 
% matrix S.  
%
% V is 1x3 in the case that S (3x3) = 
%
%               |  0  -v3  v1 |
%               | v3    0  v2 |
%               |  0    0   0 |
%
% V is 1x6 in the case that S (6x6) = 
%
%
%               |  0  -v6   v5  v1 |
%               | v6    0  -v4  v2 |
%               |-v5   v4    0  v3 |
%               |  0    0    0   0 |
%
% Notes::
% - This is the inverse of the function SKEWA().
% - The matrices are the generator matrices for se(2) and se(3).
% - This function maps se(2) and se(3) to twist vectors.
%
% References::
% - Robotics, Vision & Control: Second Edition, Chap 2,
%   P. Corke, Springer 2016.
%
% See also SKEWA, VEX, Twist.


% Copyright (C) 1993-2017, by Peter I. Corke
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

function s = vexa(Omega)

    if all(size(Omega) == [4 4])
        s = [transl(Omega); vex(Omega(1:3,1:3))];
    elseif all(size(Omega) == [3 3 ])
        s = [transl2(Omega); vex(Omega(1:2,1:2))];
    else
        error('RTB:vexa:badarg', 'argument must be a 3x3 or 4x4 matrix');
    end
    
    
end