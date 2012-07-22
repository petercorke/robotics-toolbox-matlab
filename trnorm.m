%TRNORM Normalize a homogeneous transform
%
% TN = TRNORM(T) is a normalized homogeneous transformation matrix in which 
% the rotation submatrix R = [N,O,A] is guaranteed to be a proper orthogonal 
% matrix. The O and A vectors are normalized and the normal vector is formed from
% N = O x A, and then we ensure that O and A are orthogonal by O = A x N.
%
% Notes::
% - Used to prevent finite word length arithmetic causing transforms to 
%   become `unnormalized'.
%
% See also OA2TR.


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

