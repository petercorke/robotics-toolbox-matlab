%MDL_IRB140 Create model of ABB IRB 140 manipulator
%
% MDL_IRB140 is a script that creates the workspace variable robot which
% describes the kinematic characteristics of an ABB IRB 140 manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qd         lower arm horizontal as per data sheet
%
% Reference::
% - "IRB 140 data sheet", ABB Robotics.
% - "Utilizing the Functional Work Space Evaluation Tool for Assessing a 
%   System Design and Reconfiguration Alternatives"
%   A. Djuric and R. J. Urbanic
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_fanuc10l, mdl_m16, mdl_motormanHP6, mdl_S4ABB2p8, mdl_puma560.

% MODEL: ABB, IRB140, 6DOF, standard_DH


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

function r = mdl_irb140()
    
    deg = pi/180;
    
    % robot length values (metres)
    d1 = 0.352;
    a1 = 0.070;
    a2 = 0.360;
    d4 = 0.380;
    d6 = 0.065;
    
    % DH parameter table
    %     theta d a alpha
    dh = [0 d1 a1  -pi/2
          0 0  a2  0
          0 0  0   pi/2
          0 d4 0   -pi/2
          0 0  0   pi/2
          0 d6 0   pi/2];
    
    
    % and build a serial link manipulator
    
    robot = SerialLink(dh, 'name', 'IRB 140', ...
        'manufacturer', 'ABB', 'ikine', 'nooffset'); 
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('base', 'irb140', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('base', 'qd', [0 -90 180 0 0 -90]*deg); % data sheet pose, horizontal
        assignin('base', 'qr', [0 -90 90 0 90 -90]*deg); % ready pose, arm up
    end
end

