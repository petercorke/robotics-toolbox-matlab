%ANGVEC2TR Convert angle and vector orientation to a homogeneous transform
%
% T = ANGVEC2TR(THETA, V) is a homogeneous transform matrix (4x4) equivalent to a
% rotation of THETA about the vector V.
%
% Note::
% - The translational part is zero.
% - If THETA == 0 then return identity matrix.
% - If THETA ~= 0 then V must have a finite length.
%
% See also angvec2r, eul2tr, rpy2tr, angvec2r, tr2angvec, trexp, SO3.angvec.


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

function T = angvec2tr(theta, k)

    assert( nargin >= 2, 'RTB:angvec2tr:badarg', 'two arguments required');

    T = r2t( angvec2r(theta, k) );
end
