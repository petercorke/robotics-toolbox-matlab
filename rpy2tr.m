%RPY2TR Roll-pitch-yaw angles to homogeneous transform
%
% T = RPY2TR(ROLL, PITCH, YAW, OPTIONS) is an SE(3) homogeneous
% transformation matrix (4x4) equivalent to the specified roll, pitch, yaw
% angles angles. These correspond to rotations about the X, Y, Z axes
% respectively. If ROLL, PITCH, YAW are column vectors (Nx1) then they are
% assumed to represent a trajectory and R is a three-dimensional matrix
% (4x4xN), where the last index corresponds to rows of ROLL, PITCH, YAW.
%
% T = RPY2TR(RPY, OPTIONS) as above but the roll, pitch, yaw angles angles
% angles are taken from consecutive columns of the passed matrix RPY =
% [ROLL, PITCH, YAW].  If RPY is a matrix (Nx3) then they are assumed to
% represent a trajectory and T is a three-dimensional matrix (4x4xN), where
% the last index corresponds to rows of RPY which are assumed to be [ROLL,
% PITCH, YAW].
%
% Options::
%  'deg'   Compute angles in degrees (radians default)
%  'zyx'   Return solution for sequential rotations about Z, Y, X axes (Paul book)
%
% Note::
% - In previous releases (<8) the angles corresponded to rotations about ZYX. Many 
%   texts (Paul, Spong) use the rotation order ZYX. This old behaviour can be enabled 
%   by passing the option 'zyx'
%
% See also TR2RPY, RPY2R, EUL2TR.


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

function T = rpy2tr(roll, varargin)

    R = rpy2r(roll, varargin{:});
    T = r2t(R);
