%RPY2R Roll-pitch-yaw angles to rotation matrix
%
% R = RPY2R(RPY, OPTIONS) is an orthonormal rotation matrix equivalent to the 
% specified roll, pitch, yaw angles which correspond to rotations about the 
% X, Y, Z axes respectively. If RPY has multiple rows they are assumed to 
% represent a trajectory and R is a three dimensional matrix, where the last index  
% corresponds to the rows of RPY.
%
% R = RPY2R(ROLL, PITCH, YAW, OPTIONS) as above but the roll-pitch-yaw angles 
% are passed as separate arguments. If ROLL, PITCH and YAW are column vectors 
% they are assumed to represent a trajectory and R is a three dimensional matrix,
% where the last index corresponds to the rows of ROLL, PITCH, YAW.
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
% See also TR2RPY, EUL2TR.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function R = rpy2r(roll, varargin)
    opt.zyx = false;
    opt.deg = false;
    [opt,args] = tb_optparse(opt, varargin);

    % unpack the arguments
    if numcols(roll) == 3
		pitch = roll(:,2);
		yaw = roll(:,3);
		roll = roll(:,1);
	elseif nargin >= 3
        pitch = args{1};
        yaw = args{2};
    else
        error('bad arguments')
    end

    % optionally convert from degrees
    if opt.deg
        d2r = pi/180.0;
        roll = roll * d2r;
        pitch = pitch * d2r;
        yaw = yaw * d2r;
    end

    if ~opt.zyx
        % XYZ order
        if numrows(roll) == 1
            R = rotx(roll) * roty(pitch) * rotz(yaw);
        else
            R = zeros(3,3,numrows(roll));
            for i=1:numrows(roll)
                R(:,:,i) = rotx(roll(i)) * roty(pitch(i)) * rotz(yaw(i));
            end
        end
    else
        % old ZYX order (as per Paul book)
        if numrows(roll) == 1
            R = rotz(roll) * roty(pitch) * rotx(yaw);
        else
            R = zeros(3,3,numrows(roll));
            for i=1:numrows(roll)
                R(:,:,i) = rotz(roll(i)) * roty(pitch(i)) * rotx(yaw(i));
            end
        end
    end
