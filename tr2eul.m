%TR2EUL Convert homogeneous transform to Euler angles
%
% EUL = TR2EUL(T, OPTIONS) are the ZYZ Euler angles (1x3) corresponding to
% the rotational part of a homogeneous transform T (4x4). The 3 angles
% EUL=[PHI,THETA,PSI] correspond to sequential rotations about the Z, Y and
% Z axes respectively.
%
% EUL = TR2EUL(R, OPTIONS) as above but the input is an orthonormal
% rotation matrix R (3x3).
%
% If R (3x3xK) or T (4x4xK) represent a sequence then each row of EUL
% corresponds to a step of the sequence.
%
% Options::
%  'deg'      Compute angles in degrees (radians default)
%  'flip'     Choose first Euler angle to be in quadrant 2 or 3.
%
% Notes::
% - There is a singularity for the case where THETA=0 in which case PHI is arbitrarily
%   set to zero and PSI is the sum (PHI+PSI).
% - Translation component is ignored.
%
% See also  EUL2TR, TR2RPY.




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

function euler = tr2eul(RR, varargin)
    
    opt.deg = false;
    opt.flip = false;
    opt = tb_optparse(opt, varargin);
    
    RR = SO3.check(RR);
    euler = zeros(length(RR),3);
    
    for i=1:length(RR)
        
        R = RR(i).R;
        
        % Method as per Paul, p 69.
        % euler = [phi theta psi]
        %
        
        if abs(R(1,3)) < eps && abs(R(2,3)) < eps
            % singularity
            eul(1) = 0;
            sp = 0;
            cp = 1;
            eul(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
            eul(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
        else
            % non singular
            
            % Only positive phi is returned.
            if opt.flip
                eul(1) = atan2(-R(2,3), -R(1,3));
            else
                eul(1) = atan2(R(2,3), R(1,3));
            end
            sp = sin(eul(1));
            cp = cos(eul(1));
            eul(2) = atan2(cp*R(1,3) + sp*R(2,3), R(3,3));
            eul(3) = atan2(-sp * R(1,1) + cp * R(2,1), -sp*R(1,2) + cp*R(2,2));
        end
        
        euler(i,:) = eul;
    end
    
    if opt.deg
        euler = euler * 180/pi;
    end
end