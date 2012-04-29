%T2R Return rotational submatrix of a homogeneous transformation
%
% R = T2R(T) is the orthonormal rotation matrix component of homogeneous 
% transformation matrix T:
%
% Notes::
% - Works for T in SE(2) or SE(3)
%   - If T is 4x4, then R is 3x3.
%   - If T is 3x3, then R is  2x2.
% - The validity of rotational part is not checked
% - For a homogeneous transform sequence returns a rotation matrix sequence
%
% See also R2T, TR2RT, RT2TR.


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

function R = t2r(T)
  
    % check dimensions: T is SE(2) or SE(3)
    d = size(T);
    if d(1) ~= d(2)
        error('RTB:t2r:badarg', 'matrix must be square');
    end
    if ~any(d(1) == [3 4])
        error('RTB:t2r:badarg', 'argument is not a homogeneous transform (sequence)');
    end
    
    n = d(1);     % works for SE(2) or SE(3)

    if numel(d) == 2
        % single matrix case
        R = T(1:n-1,1:n-1);
    else
        %  matrix sequence case
        R = zeros(3,3,d(3));
        for i=1:d(3)
            R(:,:,i) = T(1:n-1,1:n-1,i);
        end
    end
