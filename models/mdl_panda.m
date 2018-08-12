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
    
    % robot length values (metres)
    d = [333 0 316 0 384 0 0]'*mm;

    a = [0 0 0 82.5 -82.5 0 88]'*mm;

    alpha = [0 -pi/2 pi/2 pi/2 -pi/2 pi/2 pi/2]';
    
    theta = zeros(7,1);
    
    DH = [theta d a alpha];

    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.
    qz = [0 0 0 0 0 0 0];
    qr = [0 -90 -90 90 0 -90 90]*deg;
    
    robot = SerialLink(DH, ...
        'configs', {'qz', qz, 'qr', qr}, ...
        'name', 'PANDA', 'manufacturer', 'Franka-Emika');
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'panda', robot);
        assignin('caller', 'qz', qz); % zero angles
        assignin('caller', 'qr', qr); 
    end
end
