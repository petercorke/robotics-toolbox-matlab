%RPY2TR Roll-pitch-yaw angles to homogeneous transform
%
% T = RPY2TR(ROLL, PITCH, YAW, OPTIONS) is an SE(3) homogeneous
% transformation matrix (4x4) with zero translation and rotation equivalent
% to the specified roll, pitch, yaw angles angles. These correspond to
% rotations about the Z, Y, X axes respectively. If ROLL, PITCH, YAW are
% column vectors (Nx1) then they are assumed to represent a trajectory and
% R is a three-dimensional matrix (4x4xN), where the last index corresponds
% to rows of ROLL, PITCH, YAW.
%
% T = RPY2TR(RPY, OPTIONS) as above but the roll, pitch, yaw angles are
% taken from the vector (1x3) RPY=[ROLL,PITCH,YAW]. If RPY is a matrix
% (Nx3) then R is a three-dimensional matrix (4x4xN), where the last index
% corresponds to rows of RPY which are assumed to be
% ROLL,PITCH,YAW].
%
% Options::
%  'deg'      Compute angles in degrees (radians default)
%  'xyz'      Rotations about X, Y, Z axes (for a robot gripper)
%  'zyx'      Rotations about Z, Y, X axes (for a mobile robot, default)
%  'yxz'      Rotations about Y, X, Z axes (for a camera)
%  'arm'      Rotations about X, Y, Z axes (for a robot arm)
%  'vehicle'  Rotations about Z, Y, X axes (for a mobile robot)
%  'camera'   Rotations about Y, X, Z axes (for a camera)
%
% Note::
% - Toolbox rel 8-9 has the reverse angle sequence as default.
% - ZYX order is appropriate for vehicles with direction of travel in the X
%   direction.  XYZ order is appropriate if direction of travel is in the Z
%   direction.
% - 'arm', 'vehicle', 'camera' are synonyms for 'xyz', 'zyx' and 'yxz'
%   respectively.
%
% See also TR2RPY, RPY2R, EUL2TR.



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

function T = rpy2tr(roll, varargin)

    R = rpy2r(roll, varargin{:});
    T = r2t(R);
