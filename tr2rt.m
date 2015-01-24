%TR2RT Convert homogeneous transform to rotation and translation 
%
% [R,t] = TR2RT(TR) splits a homogeneous transformation matrix (NxN) into an 
% orthonormal rotation matrix R (MxM) and a translation vector t (Mx1), where
% N=M+1.
%
% Works for TR in SE(2) or SE(3)
%  - If TR is 4x4, then R is 3x3 and T is 3x1.
%  - If TR is 3x3, then R is 2x2 and T is 2x1.
%
% A homogeneous transform sequence TR (NxNxK) is split into rotation matrix 
% sequence R (MxMxK) and a translation sequence t (KxM).
%
%
% Notes::
% - The validity of R is not checked.
%
% See also RT2TR, R2T, T2R.


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

function [R,t] = tr2rt(T)
    if numcols(T) ~= numrows(T)
        error('T must be square');
    end

    n = numcols(T);

    if size(T,3) > 1
        R = zeros(3,3,size(T,3));
        t = zeros(size(T,3), 3);
        for i=1:size(T,3)
            R(:,:,i) = T(1:n-1,1:n-1,i);
            t(i,:) = T(1:n-1,n,i)';
        end
    else
        R = T(1:n-1,1:n-1);
        t = T(1:n-1,n);
    end
