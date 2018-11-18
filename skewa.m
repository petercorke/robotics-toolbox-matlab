%SKEWA Create augmented skew-symmetric matrix
%
% S = SKEWA(V) is an augmented skew-symmetric matrix formed from V.
% 
% If V (1x3) then S =
%
%           |  0  -v3  v1 |
%           | v3    0  v2 |
%           |  0    0   0 |
%
% and if V (1x6) then S =
%
%           |  0  -v6   v5  v1 |
%           | v6    0  -v4  v2 |
%           |-v5   v4    0  v3 |
%           |  0    0    0   0 |
%
% Notes::
% - This is the inverse of the function VEXA().
% - These are the generator matrices for the Lie algebras se(2) and se(3).
% - Map twist vectors in 2D and 3D space to se(2) and se(3).
%
% References::
% - Robotics, Vision & Control: Second Edition, Chap 2,
%   P. Corke, Springer 2016.
%
% See also SKEW, VEX, Twist.


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

function Omega = skewa(s)
    s  = s(:);
    switch length(s)
        case 3
            Omega = [skew(s(3)) s(1:2); 0 0 0];
            
        case 6
            Omega = [skew(s(4:6)) s(1:3); 0 0 0 0];
            
        otherwise
            error('RTB:skewa:badarg', 'expecting a 3- or 6-vector');
    end
end