%TRNORM Normalize a rotation matrix
%
% RN = TRNORM(R) is guaranteed to be a proper orthogonal matrix rotation
% matrix (3x3) which is "close" to the non-orthogonal matrix R (3x3). If R
% = [N,O,A] the O and A vectors are made unit length and the normal vector
% is formed from N = O x A, and then we ensure that O and A are orthogonal
% by O = A x N.
%
% TN = TRNORM(T) as above but the rotational submatrix of the homogeneous
% transformation T (4x4) is normalised while the translational part is
% passed unchanged.
%
% If R (3x3xK) or T (4x4xK) represent a sequence then RN and TN have the
% same dimension and normalisation is performed on each plane.
%
% Notes::
% - Only the direction of A (the z-axis) is unchanged.
% - Used to prevent finite word length arithmetic causing transforms to 
%   become `unnormalized'.
%
% See also OA2TR, SO3.trnorm, SE3.trnorm.



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

function TR = trnorm(T)

    assert(ishomog(T) || isrot(T), 'RTB:trnorm:badarg', 'expecting 3x3xN or 4x4xN hom xform');
    
    if ndims(T) == 3
        % recurse for transform sequence
        nd = size(T, 3);
        r = zeros(4,4,nd);
        for i=1:nd
            TR(:,:,i) = trnorm(T(:,:,i));
        end
        return
    end
    
    n = T(1:3,1); o = T(1:3,2); a = T(1:3,3);
    n = cross(o, a);         % N = O x A
    o = cross(a, n);         % O = A x N
    R = [unit(n) unit(o) unit(a)];
    
    if ishomog(T)
        TR = rt2tr( R, T(1:3,4) );
    elseif isrot(T)
        TR = R;
    end

