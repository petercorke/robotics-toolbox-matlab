%ANGVEC2R Convert angle and vector orientation to a rotation matrix
%
% R = ANGVEC2R(THETA, V) is an orthonormal rotation matrix (3x3)
% equivalent to a rotation of THETA about the vector V.
%
% Notes::
% - If THETA == 0 then return identity matrix.
% - If THETA ~= 0 then V must have a finite length.
%
% See also angvec2tr, eul2r, rpy2r, tr2angvec, trexp, SO3.angvec.


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
function R = angvec2r(theta, v)

    if nargin < 2 || ~isscalar(theta) || ~isvec(v)
        error('RTB:angvec2r:badarg', 'bad arguments');
    end
    if ~isa(v, 'sym') && norm(v) < 10*eps
        if (abs(theta) > 0)
            error('RTB:angvec2r:badarg', 'norm of direction is zero');
        else
            R = eye(3,3);
            return;
        end
    end
   
    % Rodrigue's equation
    
    sk = skew( unit(v) );
    R = eye(3,3) + sin(theta)*sk + (1-cos(theta))*sk^2;
end
