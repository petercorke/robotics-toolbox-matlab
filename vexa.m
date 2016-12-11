%VEXA Convert augmented skew-symmetric matrix to vector
%
% V = VEXA(S) is the vector which has the corresponding augmented skew-symmetric 
% matrix S.  
%
% In the case that S (3x3) then V is 1x3
%
%           S = |  0  -v1  v2 |
%               | v1    0  v3 |
%               |  0    0   0 |
%
%In the case that S (6x6) then V is 1x6
%
%
%               |  0  -v3   v2  v4 |
%           S = | v3    0  -v1  v5 |
%               |-v2   v1    0  v6 |
%
% Notes::
% - The matrices are the generator matrices for se(2) and se(3).
% - This function maps se(2) and se(3) to twist vectors.
%
% References::
% - Robotics, Vision & Control: Second Edition, Chap 2,
%   P. Corke, Springer 2016.
%
% See also SKEWA, VEX, Twist.

% Copyright (C) 1993-2016, by Peter I. Corke
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
        if ~all(size(Omega) == [4 4])
        error('RTB:vexa:badarg', 'expecting a 4x4 matrix');
        end
    
        s = [transl(Omega); vex(Omega(1:3,1:3))];
end