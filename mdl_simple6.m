%MDL_SIMPLE6 A minimalistic 6DOF robot arm
%
% MDL_SIMPLE6 is a script creates the workspace variable s6 which describes
% the kinematic characteristics of a simple arm manipulator with a
% spherical wrist and no shoulder offset, using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Notes::
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_offset6, mdl_puma560.

% MODEL: generic, 6DOF, standard_DH

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

function r = mdl_simple6()
    
    % robot length values (metres)
    L1 = 1;
    L2 = 1;
    
    % and build a serial link manipulator
    
    robot = SerialLink([
        Revolute('alpha', pi/2, 'a', 0,  'd', 0)
        Revolute('alpha', 0,     'a', L1, 'd', 0)
        Revolute('alpha', -pi/2,  'a', L2, 'd', 0)
        Revolute('alpha', -pi/2, 'a', 0,  'd', 0)
        Revolute('alpha', pi/2,  'a', 0,  'd', 0)
        Revolute('alpha', 0,     'a', 0,  'd', 0)
        ], ...
        'name', 'Simple6');
    
    % place the variables into the global workspace
    if nargout == 1
        r = robot;
    elseif nargout == 0
        assignin('base', 's6', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles, arm up
    end
end
