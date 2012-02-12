%TR2JAC Jacobian for differential motion
%
% J = TR2JAC(T) is a Jacobian matrix (6x6) that maps spatial velocity or
% differential motion from the world frame to the frame represented by 
% the homogeneous transform T.
%
% See also WTRANS, TR2DELTA, DELTA2TR.


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

function J = tr2jac(T)
		
    R = t2r(T);
    J = [
                 R'  (skew(transl(T))*R)'
        zeros(3,3)                     R'
        ];
