%ANGVEC2TR Convert angle and vector orientation to a homogeneous transform
%
% T = ANGVEC2TR(THETA, V) is a homogeneous transform matrix equivalent to a 
% rotation of THETA about the vector V.
%
% Note::
% - The translational part is zero.
%
% See also EUL2TR, RPY2TR, ANGVEC2R.


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

function T = angvec2tr(theta, k)

    if nargin < 2 
        error('RTB:angvec2tr:badarg', 'bad arguments');
    end


    T = r2t( angvec2r(theta, k) );
