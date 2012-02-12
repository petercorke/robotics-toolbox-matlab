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
function qq = mrdivide(q1, q2)
%Quaternion.mrdivide Compute quaternion quotient.
%
% Q1/Q2   is a quaternion formed by Hamilton product of Q1 and inv(Q2)
% Q/S     is the element-wise division of quaternion elements by by the scalar S

    if isa(q2, 'Quaternion')
        % qq = q1 / q2
        %    = q1 * qinv(q2)

        qq = q1 * inv(q2);
    elseif isa(q2, 'double')
        qq = Quaternion( double(q1) / q2 );
    end
end
