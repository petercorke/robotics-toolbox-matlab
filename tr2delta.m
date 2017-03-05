%TR2DELTA Convert homogeneous transform to differential motion
%
% D = TR2DELTA(T0, T1) is the differential motion (6x1) corresponding to 
% infinitessimal motion (in the T0 frame) from pose T0 to T1 which are homogeneous 
% transformations (4x4) or SE3 objects. D=(dx, dy, dz, dRx, dRy, dRz). 
%
% D = TR2DELTA(T) as above but the motion is with respect to the world frame.
%
% Notes::
% - D is only an approximation to the motion T, and assumes
%   that T0 ~ T1 or T ~ eye(4,4).
% - can be considered as an approximation to the effect of spatial velocity over a
%   a time interval, average spatial velocity multiplied by time.
%
% Reference::
% - Robotics, Vision & Control 2nd Edition, p67
%
% See also DELTA2TR, SKEW.



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

function delta = tr2delta(A, B)

    % handle arguments, convert all to 4x4 matrices
    if nargin > 0
        if isa(A, 'SE3')
            T1 = A.double;
        elseif ishomog(A)
            T1 = A;
        else
            error('RTB:tr2delta:badarg', 'T1 should be a homogeneous transformation');
        end
        T0 = eye(4,4);
    end
    if nargin > 1
        T0 = T1;
        if isa(B, 'SE3')
            T1 = B.double;
        elseif ishomog(B)
            T1 = B;
        else
                error('RTB:tr2delta:badarg', 'T0 should be a homogeneous transformation');
        end
    end
    
    % compute incremental transformation from T0 to T1 in the T0 frame
    TD = inv(T0) * T1;

    % build the delta vector
    delta = [transl(TD); vex(t2r(TD) - eye(3,3))];
    
    
%    R0 = t2r(T0); R1 = t2r(T1);
%    % in world frame
%    %[th,vec] = tr2angvec(R1*R0');
%    dR = vex(R1*R0');
%    %delta = [ (T1(1:3,4)-T0(1:3,4)); th*vec' ];
%    delta = [ (T1(1:3,4)-T0(1:3,4)); dR];

% same as above but more complex
%    delta = [	T1(1:3,4)-T0(1:3,4);
%        0.5*(	cross(T0(1:3,1), T1(1:3,1)) + ...
%            cross(T0(1:3,2), T1(1:3,2)) + ...
%            cross(T0(1:3,3), T1(1:3,3)) ...
%        )];
end

