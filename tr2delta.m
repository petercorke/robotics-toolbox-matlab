%TR2DELTA Convert homogeneous transform to differential motion
%
% D = TR2DELTA(T0, T1) is the differential motion (6x1) corresponding to 
% infinitessimal motion from pose T0 to T1 which are homogeneous 
% transformations (4x4). D=(dx, dy, dz, dRx, dRy, dRz) and is an approximation
% to the average spatial velocity multiplied by time.
%
% D = TR2DELTA(T) is the differential motion corresponding to the
% infinitessimal relative pose T expressed as a homogeneous 
% transformation.
%
% Notes::
% - D is only an approximation to the motion T, and assumes
%   that T0 ~ T1 or T ~ eye(4,4).
%
% See also DELTA2TR, SKEW.



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

function delta = tr2delta(T0, T1)
    if nargin == 1
        T1 = T0;
        T0 = eye(4,4);
    end
    R0 = t2r(T0); R1 = t2r(T1);
    % in world frame
    delta = [ (T1(1:3,4)-T0(1:3,4)); vex( R1*R0' - eye(3,3)) ];
    % in T0 frame
    %delta = [ R0'*(T1(1:3,4)-T0(1:3,4)); R0'*vex( R1*R0' - eye(3,3)) ];

% TODO HACK understand/fix this and update Chapter 2
%    delta = [	T1(1:3,4)-T0(1:3,4);
%        0.5*(	cross(T0(1:3,1), T1(1:3,1)) + ...
%            cross(T0(1:3,2), T1(1:3,2)) + ...
%            cross(T0(1:3,3), T1(1:3,3)) ...
%        )];
end

