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
function qp = mpower(q, p)
%Quaternion.mpower Raise quaternion to integer power
%
% Q^N is quaternion Q raised to the integer power N, and computed by repeated multiplication.

    % check that exponent is an integer
    if (p - floor(p)) ~= 0
        error('quaternion exponent must be integer');
    end

    qp = q;

    % multiply by itself so many times
    for i = 2:abs(p)
        qp = qp * q;
    end

    % if exponent was negative, invert it
    if p<0
        qp = inv(qp);
    end
end
