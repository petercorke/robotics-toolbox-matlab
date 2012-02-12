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
function q = interp(Q1, Q2, r)
%Quaternion.interp Interpolate rotations expressed by quaternion objects
%
% QI = Q1.interp(Q2, R) is a unit-quaternion that interpolates between Q1 for R=0 
% to Q2 for R=1. This is a spherical linear interpolation (slerp) that can be 
% interpretted as interpolation along a great circle arc on a sphere.
%
% If R is a vector QI is a vector of quaternions, each element
% corresponding to sequential elements of R.
%
% Notes:
% - the value of r is clipped to the interval 0 to 1
%
% See also ctraj, Quaternion.scale.

    q1 = double(Q1);
    q2 = double(Q2);

    theta = acos(q1*q2');
    count = 1;

    % clip values of r
    r(r<0) = 0;
    r(r>1) = 1;

    if length(r) == 1
        if theta == 0
            q = Q1;
        else
            q = Quaternion( (sin((1-r)*theta) * q1 + sin(r*theta) * q2) / sin(theta) );
        end
    else
        for R=r(:)'
            if theta == 0
                qq = Q1;
            else
                qq = Quaternion( (sin((1-R)*theta) * q1 + sin(R*theta) * q2) / sin(theta) );
            end
            q(count) = qq;
            count = count + 1;
        end
    end
end
