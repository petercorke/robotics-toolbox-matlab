%TR2EUL Convert homogeneous transform to Euler angles
%
% EUL = TR2EUL(T, OPTIONS) are the ZYZ Euler angles expressed as a row vector
% corresponding to the rotational part of a homogeneous transform T.
% The 3 angles EUL=[PHI,THETA,PSI] correspond to sequential rotations about 
% the Z, Y and Z axes respectively.
%
% EUL = TR2EUL(R, OPTIONS) are the ZYZ Euler angles expressed as a row vector
% corresponding to the orthonormal rotation matrix R.
%
% If R or T represents a trajectory (has 3 dimensions), then each row of EUL
% corresponds to a step of the trajectory.
%
% Options::
%  'deg'      Compute angles in degrees (radians default)
%
% Notes::
% - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
%   set to zero and PSI is the sum (PHI+PSI).
%
% See also  EUL2TR, TR2RPY.


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

function euler = tr2eul(m, varargin)

    opt.deg = false;
    opt = tb_optparse(opt, varargin);

	s = size(m);
	if length(s) > 2
        euler = zeros(s(3), 3);
		for i=1:s(3)
			euler(i,:) = tr2eul(m(:,:,i));
		end

        if opt.deg
            euler = euler * 180/pi;
        end
		return
	end

	euler = zeros(1,3);

	% Method as per Paul, p 69.
	% phi = atan2(ay, ax)
	% Only positive phi is returned.
	if abs(m(1,3)) < eps && abs(m(2,3)) < eps
		% singularity
		euler(1) = 0;
		sp = 0;
		cp = 1;
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
	else
		euler(1) = atan2(m(2,3), m(1,3));
		sp = sin(euler(1));
		cp = cos(euler(1));
		euler(2) = atan2(cp*m(1,3) + sp*m(2,3), m(3,3));
		euler(3) = atan2(-sp * m(1,1) + cp * m(2,1), -sp*m(1,2) + cp*m(2,2));
	end
    if opt.deg
        euler = euler * 180/pi;
    end
