%OA2TR Convert orientation and approach vectors to homogeneous transformation
%
% T = OA2TR(O, A) is a homogeneous tranformation for the specified orientation 
% and approach vectors (3x1) formed from 3 vectors such that R = [N O A] and 
% N = O x A.
%
% Notes::
% - The rotation submatrix is guaranteed to be orthonormal so long as O and A 
%   are not parallel.
% - The translational part is zero.
% - The vectors O and A are parallel to the Y- and Z-axes of the coordinate
%   frame.
%
% See also RPY2TR, EUL2TR, OA2R.


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

function r = oa2tr(o, a)
    if nargin < 2
        error('RTB:oa2tr:badarg', 'bad arguments');
    end
	n = cross(o, a);
    o = cross(a, n);
	r = [unit(n(:)) unit(o(:)) unit(a(:)) zeros(3,1); 0 0 0 1];
