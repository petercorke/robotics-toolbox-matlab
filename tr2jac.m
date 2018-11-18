%TR2JAC Jacobian for differential motion
%
% J = TR2JAC(TAB) is a Jacobian matrix (6x6) that maps spatial velocity or
% differential motion from frame {A} to frame {B} where the pose of {B}
% relative to {A} is represented by the homogeneous transform TAB (4x4).
%
% J = TR2JAC(TAB, 'samebody') is a Jacobian matrix (6x6) that maps spatial
% velocity or differential motion from frame {A} to frame {B} where both
% are attached to the same moving body.  The pose of {B} relative to {A} is
% represented by the homogeneous transform TAB (4x4).
%
% See also WTRANS, TR2DELTA, DELTA2TR, SE3.velxform.




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

function J = tr2jac(T, varargin)
    
    opt.samebody = false;
    
    opt = tb_optparse(opt, varargin);
		
    R = t2r(T);
    
    if opt.samebody
        J = [
            R'              (skew(transl(T))*R)'
            zeros(3,3)      R'
            ];
    else
        J = [
            R'              zeros(3,3)
            zeros(3,3)      R'
            ];
    end
