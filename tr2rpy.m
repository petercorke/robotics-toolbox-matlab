%TR2RPY Convert a homogeneous transform to roll-pitch-yaw angles
%
% RPY = TR2RPY(T, options) are the roll-pitch-yaw angles expressed as a row 
% vector corresponding to the rotation part of a homogeneous transform T.
% The 3 angles RPY=[R,P,Y] correspond to sequential rotations about 
% the X, Y and Z axes respectively.
%
% RPY = TR2RPY(R, options) are the roll-pitch-yaw angles expressed as a row 
% vector corresponding to the orthonormal rotation matrix R.
%
% If R or T represents a trajectory (has 3 dimensions), then each row of RPY
% corresponds to a step of the trajectory.
%
% Options::
%  'deg'   Compute angles in degrees (radians default)
%  'zyx'   Return solution for sequential rotations about Z, Y, X axes (Paul book)
%
% Notes::
% - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
%   set to zero and Y is the sum (R+Y).
% - Note that textbooks (Paul, Spong) use the rotation order ZYX.
%
% See also  rpy2tr, tr2eul.


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

% TODO singularity for XYZ case, 
function rpy = tr2rpy(m, varargin)
	
    opt.deg = false;
    opt.zyx = false;
    opt = tb_optparse(opt, varargin);

	s = size(m);
	if length(s) > 2
		rpy = zeros(s(3), 3);
		for i=1:s(3)
			rpy(i,:) = tr2rpy(m(:,:,i), varargin{:});
		end
		return
	end
	rpy = zeros(1,3);

    if ~opt.zyx
        % XYZ order
        if abs(m(3,3)) < eps && abs(m(2,3)) < eps
            % singularity
            rpy(1) = 0;  % roll is zero
            rpy(2) = atan2(m(1,3), m(3,3));  % pitch
            rpy(3) = atan2(m(2,1), m(2,2));  % yaw is sum of roll+yaw
        else
            rpy(1) = atan2(-m(2,3), m(3,3));        % roll
            % compute sin/cos of roll angle
            sr = sin(rpy(1));
            cr = cos(rpy(1));
            rpy(2) = atan2(m(1,3), cr * m(3,3) - sr * m(2,3));  % pitch
            rpy(3) = atan2(-m(1,2), m(1,1));        % yaw
        end
    else
        % old ZYX order (as per Paul book)
        if abs(m(1,1)) < eps && abs(m(2,1)) < eps
            % singularity
            rpy(1) = 0;     % roll is zero
            rpy(2) = atan2(-m(3,1), m(1,1));  % pitch
            rpy(3) = atan2(-m(2,3), m(2,2));  % yaw is difference yaw-roll
        else
            rpy(1) = atan2(m(2,1), m(1,1));
            sp = sin(rpy(1));
            cp = cos(rpy(1));
            rpy(2) = atan2(-m(3,1), cp * m(1,1) + sp * m(2,1));
            rpy(3) = atan2(sp * m(1,3) - cp * m(2,3), cp*m(2,2) - sp*m(1,2));
        end
    end
    if opt.deg
        rpy = rpy * 180/pi;
    end
end
