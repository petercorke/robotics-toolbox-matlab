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
function qp = mtimes(q1, q2)
%Quaternion.mtimes Multiply a quaternion object
%
% Q1*Q2   is a quaternion formed by Hamilton product of two quaternions.
% Q*V     is the vector V rotated by the quaternion Q
% Q*S     is the element-wise multiplication of quaternion elements by by the scalar S

    if isa(q1, 'Quaternion') & isa(q2, 'Quaternion')
    %QQMUL  Multiply unit-quaternion by unit-quaternion
    %
    %   QQ = qqmul(Q1, Q2)
    %
    %   Return a product of unit-quaternions.
    %
    %   See also: TR2Q


        % decompose into scalar and vector components
        s1 = q1.s;  v1 = q1.v;
        s2 = q2.s;  v2 = q2.v;

        % form the product
        qp = Quaternion([s1*s2-v1*v2' s1*v2+s2*v1+cross(v1,v2)]);

    elseif isa(q1, 'Quaternion') & isa(q2, 'double')

    %QVMUL  Multiply vector by unit-quaternion
    %
    %   VT = qvmul(Q, V)
    %
    %   Rotate the vector V by the unit-quaternion Q.
    %
    %   See also: QQMUL, QINV

        if length(q2) == 3
			qp = q1 * Quaternion([0 q2(:)']) * inv(q1);
            qp = qp.v(:);
        elseif length(q2) == 1
            qp = Quaternion(double(q1)*q2);
        else
            error('quaternion-vector product: must be a 3-vector or scalar');
        end

    elseif isa(q2, 'Quaternion') & isa(q1, 'double')
        if length(q1) == 3
            qp = q2 * Quaternion([0 q1(:)']) * inv(q2);
            qp = qp.v;
        elseif length(q1) == 1
            qp = Quaternion(double(q2)*q1);
        else
            error('quaternion-vector product: must be a 3-vector or scalar');
        end
    end
end
