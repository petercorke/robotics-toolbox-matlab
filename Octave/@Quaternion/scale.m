
% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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
function q = scale(Q, r)
%Quaternion.scale Interpolate rotations expressed by quaternion objects
%
% QI = Q.scale(R) is a unit-quaternion that interpolates between identity for R=0
% to Q for R=1.  This is a spherical linear interpolation (slerp) that can
% be interpretted as interpolation along a great circle arc on a sphere.
%
% If R is a vector QI is a cell array of quaternions, each element
% corresponding to sequential elements of R.
%
% See also ctraj, Quaternion.interp.


    q2 = double(Q);

    if any(r<0) || (r>1)
        error('r out of range');
    end
    q1 = [1 0 0 0];         % identity quaternion
    theta = acos(q1*q2');

    if length(r) == 1
        if theta == 0
            q = Q;
        else
            q = unit(Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ));
        end
    else
        count = 1;
        for R=r(:)'
            if theta == 0
                qq = Q;
            else
                qq = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) ).unit;
            end
            q(count) = qq;
            count = count + 1;
        end
    end
end
