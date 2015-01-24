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
% See also OA2TR.



% Copyright (C) 1993-2015, by Peter I. Corke
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

function r = trnorm(t)

    if ndims(t) == 3
        nd = size(t, 3);
        r = zeros(4,4,nd);
        for i=1:nd
            r(:,:,i) = trnorm(t(:,:,i));
        end
        return
    end

    if all(size(t) == [4 4])
        n = cross(t(1:3,2), t(1:3,3));  % N = O x A
        o = cross(t(1:3,3), n);         % O = A x N
        r = [unit(n) unit(t(1:3,2)) unit(t(1:3,3)) t(1:3,4); 0 0 0 1];
    elseif all(size(t) == [3 3])
            r = t;
    else
        error('RTB:trnorm:badarg', 'argument must be 3x3 or 4x4 hom xform');
    end

