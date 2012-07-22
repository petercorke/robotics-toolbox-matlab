%OA2R Convert orientation and approach vectors to rotation matrix
%
% R = OA2R(O, A) is a rotation matrix for the specified orientation and 
% approach vectors (3x1) formed from 3 vectors such that R = [N O A] and 
% N = O x A.
%
% Notes::
% - The submatrix is guaranteed to be orthonormal so long as O and A 
%   are not parallel.
% - The vectors O and A are parallel to the Y- and Z-axes of the coordinate
%   frame.
%
% See also RPY2R, EUL2R, OA2TR.


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

function R = oa2r(o, a)

    if nargin < 2 || ~isvec(o) || ~isvec(a)
        error('RTB:oa2r:badarg', 'bad arguments');
    end

    o = o(:); a = a(:);
	n = cross(o, a);
    o = cross(a, n);
	R = [unit(n(:)) unit(o(:)) unit(a(:))];
