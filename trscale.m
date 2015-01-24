%TRSCALE Homogeneous transformation for pure scale
%
% T = TRSCALE(S) is a homogeneous transform (4x4) corresponding to a pure
% scale change.  If S is a scalar the same scale factor is used for x,y,z,
% else it can be a 3-vector specifying scale in the x-, y- and
% z-directions.
%


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

function t = trscale(sx, sy, sz)

    if length(sx) > 1
        s = sx;
    else
        if nargin == 1
            s = [sx sx sx];
        else
            s = [sx sy sz];
        end
    end
    t = r2t(diag(s));
