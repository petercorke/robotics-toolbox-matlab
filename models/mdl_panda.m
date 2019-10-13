%MDL_PANDA Create model of Franka-Emika PANDA robot
%
% MDL_PANDA is a script that creates the workspace variable panda which
% describes the kinematic characteristics of a Franka-Emika PANDA manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         arm along +ve x-axis configuration
%
% Reference::
% - http://www.diag.uniroma1.it/~deluca/rob1_en/WrittenExamsRob1/Robotics1_18.01.11.pdf
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also mdl_sawyer, SerialLink.

% MODEL: Franka-Emika, PANDA, 7DOF, standard_DH

% Copyright (C) 1993-2018, by Peter I. Corke
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

function r = mdl_panda()
    
    deg = pi/180;
    mm = 1e-3;
    
    d7 = 107*mm;
    
    %% Define links (thanks Alex Smith for this code)
    L1 = Link('revolute', 'a',     0.0, 'd', 0.333, 'alpha',   0.0, 'qlim', [-2.8973 2.8973], 'modified');
    L2 = Link('revolute', 'a',     0.0, 'd',   0.0, 'alpha', -pi/2, 'qlim', [-1.7628 1.7628], 'modified');
    L3 = Link('revolute', 'a',     0.0, 'd', 0.316, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], 'modified');
    L4 = Link('revolute', 'a',  0.0825, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-3.0718 -0.0698], 'modified');
    L5 = Link('revolute', 'a', -0.0825, 'd', 0.384, 'alpha', -pi/2, 'qlim', [-2.8973 2.8973], 'modified');
    L6 = Link('revolute', 'a',     0.0, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-0.0175 3.7525], 'modified');
    L7 = Link('revolute', 'a',   0.088, 'd',   0.0, 'alpha',  pi/2, 'qlim', [-2.8973 2.8973], 'modified');

    %% Create SerialLink object
    robot = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'PANDA', 'manufacturer', 'Franka-Emika', 'tool', transl([0 0 d7]));

    qz = [0 0 0 0 0 0 0];
    qr = [0 -90 -90 90 0 -90 90]*deg;
        
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'panda', robot);
        assignin('caller', 'qz', qz); % zero angles
        assignin('caller', 'qr', qr); 
    end
end
