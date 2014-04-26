%MDL_TWOLINK_MDH Create model of a simple 2-link mechanism with modified DH convention
%
%      mdl_twolink_mdh
%
% Script creates the workspace variable tl which describes the 
% kinematic and dynamic characteristics of a simple planar 2-link mechanism
% using modified Denavit-Hartenberg conventions.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane
%
% References::
%  - Based on Fig 3.8 (p71) of Craig (3rd edition).  
%
% See also SerialLink, mdl_twolink.



% Copyright (C) 1993-2014, by Peter I. Corke
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


a1 = 1;
a2 = 1;

% for MDH parameters we need to implement the second link as a tool
% transform

twolink = SerialLink([
        RevoluteMDH('d', 0, 'a', 0,  'alpha', 0)
        RevoluteMDH('d', 0, 'a', a1, 'alpha', 0)
    ], ...
    'tool', transl(a2, 0, 0), ...
    'name', 'two link', ...
    'comment', 'from Craig');
qz = [0 0];
qn = [pi/6, -pi/6];
