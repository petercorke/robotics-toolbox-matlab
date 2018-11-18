%VEX Convert skew-symmetric matrix to vector
%
% V = VEX(S) is the vector which has the corresponding skew-symmetric 
% matrix S.  
%
%
% In the case that S (2x2) then V is 1x1
%
%           S = | 0  -v |
%               | v   0 |
%
%In the case that S (3x3) then V is 3x1.
%
%               |  0  -vz   vy |
%           S = | vz    0  -vx |
%               |-vy   vx    0 |
%
% Notes::
% - This is the inverse of the function SKEW().
% - Only rudimentary checking (zero diagonal) is done to ensure that the 
%   matrix is actually skew-symmetric.
% - The function takes the mean of the two elements that correspond to 
%   each unique element of the matrix.
%
% References::
% - Robotics, Vision & Control: Second Edition, Chap 2,
%   P. Corke, Springer 2016.
%
% See also SKEW, VEXA.


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

function v = vex(S)
%     if trace(abs(S)) > 10*eps
%         error('RTB:vex:badarg', 'argument is not skew symmetric tr=%g', trace(abs(S)));
%     end
    if all(size(S) == [3 3])
        v = 0.5*[S(3,2)-S(2,3); S(1,3)-S(3,1); S(2,1)-S(1,2)];
    elseif all(size(S) == [2 2])
        v = 0.5*(S(2,1)-S(1,2));
    else
        error('RTB:vex:badarg', 'argument must be a 2x2 or 3x3 matrix');
    end
