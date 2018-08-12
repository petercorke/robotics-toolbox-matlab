%TR2RPY Convert a homogeneous transform to roll-pitch-yaw angles
%
% RPY = TR2RPY(T, options) are the roll-pitch-yaw angles (1x3)
% corresponding to the rotation part of a homogeneous transform T. The 3
% angles RPY=[R,P,Y] correspond to sequential rotations about the Z, Y and
% X axes respectively. Roll and yaw angles in [-pi,pi) while pitch angle
% in [-pi/2,pi/2).
%
% RPY = TR2RPY(R, options) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then each row of RPY
% corresponds to a step of the sequence.
%
% Options::
%  'deg'   Compute angles in degrees (radians default)
%  'xyz'      Return solution for sequential rotations about X, Y, Z axes
%  'zyx'      Return solution for sequential rotations about Z, Y, X axes (default)
%  'yxz'      Return solution for sequential rotations about Y, X, Z axes
%  'arm'      Return solution for sequential rotations about X, Y, Z axes
%  'vehicle'  Return solution for sequential rotations about Z, Y, X axes
%  'camera'   Return solution for sequential rotations about Y, X, Z axes
%
% Notes::
% - There is a singularity for the case where P=pi/2 in which case R is arbitrarily
%   set to zero and Y is the sum (R+Y).
% - Translation component is ignored.
% - Toolbox rel 8-9 has XYZ angle sequence as default.
% - 'arm', 'vehicle', 'camera' are synonyms for 'xyz', 'zyx' and 'yxz'
%   respectively.
%
% See also  rpy2tr, tr2eul.


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

% TODO singularity for XYZ case,
function [rpy,order] = tr2rpy(R, varargin)
    
    opt.deg = false;
    opt.order = {'zyx', 'xyz', 'arm', 'vehicle', 'yxz', 'camera'};
    opt = tb_optparse(opt, varargin);
    
    s = size(R);
    if length(s) > 2
        rpy = zeros(s(3), 3);
        for i=1:s(3)
            rpy(i,:) = tr2rpy(R(:,:,i), varargin{:});
        end
        return
    end
    rpy = zeros(1,3);   
    
    
    assert(isrot(R) || ishomog(R), 'RTB:tr2rpy:badarg', 'argument must be a 3x3 or 4x4 matrix');
    switch opt.order
        case {'xyz', 'arm'}
            opt.order = 'xyz';
            % XYZ order
            if abs(abs(R(1,3)) - 1) < eps  % when |R13| == 1
                % singularity
                rpy(1) = 0;  % roll is zero
                if R(1,3) > 0
                rpy(3) = atan2( R(3,2), R(2,2));   % R+Y
                else
                     rpy(3) = -atan2( R(2,1), R(3,1));   % R-Y
                end
                rpy(2) = asin(R(1,3));
            else
                rpy(1) = -atan2(R(1,2), R(1,1));
                rpy(3) = -atan2(R(2,3), R(3,3));
                
                rpy(2) = atan(R(1,3)*cos(rpy(1))/R(1,1));
            end
            
        case {'zyx', 'vehicle'}
            opt.order = 'zyx';
            % old ZYX order (as per Paul book)
            if abs(abs(R(3,1)) - 1) < eps  % when |R31| == 1
                % singularity

                rpy(1) = 0;     % roll is zero
                if R(3,1) < 0
                    rpy(3) = -atan2(R(1,2), R(1,3));  % R-Y
                else
                    rpy(3) = atan2(-R(1,2), -R(1,3));  % R+Y
                end
                rpy(2) = -asin(R(3,1));
            else
                rpy(1) = atan2(R(3,2), R(3,3));  % R
                rpy(3) = atan2(R(2,1), R(1,1));  % Y
                
                rpy(2) = -atan(R(3,1)*cos(rpy(1))/R(3,3));
            end
            
        case {'yxz', 'camera'}
            opt.order = 'yxz';
            if abs(abs(R(2,3)) - 1) < eps  % when |R23| == 1
                % singularity
                
                rpy(1) = 0;
                if R(2,3) < 0
                    rpy(3) = -atan2(R(3,1), R(1,1));   % R-Y
                else
                    rpy(3) = atan2(-R(3,1), -R(3,2));   % R+Y
                end
                
                rpy(2) = -asin(R(2,3));    % P
            else
                rpy(1) = atan2(R(2,1), R(2,2));
                rpy(3) = atan2(R(1,3), R(3,3));
                
                rpy(2) = -atan(cos(rpy(1))*R(2,3)/R(2,2));
            end
            
    end
    if opt.deg
        rpy = rpy * 180/pi;
    end
    if nargout > 1
        order = opt.order;
    end
end
