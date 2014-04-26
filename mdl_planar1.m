%MDL_PLANAR1 Create model of a simple planar 1-link mechanism
%
%      mdl_planar1
%
% Script creates the workspace variable p1 which describes the 
% kinematic and dynamic characteristics of a simple planar 1-link mechanism.
%
% Also defines the vector:
%   qz   corresponds to the zero joint angle configuration.
%
% Notes::
% - It is a planar mechanism operating in the XY (horizontal) plane and is 
%   therefore not affected by gravity.
% - No dynamics in this model
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

p1 = SerialLink([
    Revolute('d', 0, 'a', a1, 'alpha', 0, 'standard')
    ], ...
    'name', 'one link');
qz = [0];
