%EUL2TR Convert Euler angles to homogeneous transform
%
% T = EUL2TR(PHI, THETA, PSI, OPTIONS) is a SE(3) homogeneous
% transformation matrix (4x4) equivalent to the specified Euler angles.
% These correspond to rotations about the Z, Y, Z axes respectively. If
% PHI, THETA, PSI are column vectors (Nx1) then they are assumed to
% represent a trajectory and R is a three-dimensional matrix (4x4xN), where
% the last index corresponds to rows of PHI, THETA, PSI.
%
% T = EUL2TR(EUL, OPTIONS) as above but the Euler angles are taken from
% consecutive columns of the passed matrix EUL = [PHI THETA PSI].  If EUL
% is a matrix (Nx3) then they are assumed to represent a trajectory and T
% is a three-dimensional matrix (4x4xN), where the last index corresponds
% to rows of EUL which are assumed to be [PHI, THETA, PSI].
%
% Options::
%  'deg'      Compute angles in degrees (radians default)
%
% Note::
% - The vectors PHI, THETA, PSI must be of the same length.
% - The translational part is zero.
%
% See also EUL2R, RPY2TR, TR2EUL.



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

function T = eul2tr(phi, varargin)

    R = eul2r(phi, varargin{:});
    T = r2t(R);
