%MDL_UR10 Create model of Universal Robotics UR10 manipulator
%
% MDL_UR5 is a script that creates the workspace variable ur10 which
% describes the kinematic characteristics of a Universal Robotics UR10 manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         arm along +ve x-axis configuration
%
% Reference::
% - https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/actual-center-of-mass-for-robot-17264/
%
% Notes::
% - SI units of metres are used.
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also mdl_ur3, mdl_ur5, mdl_puma560, SerialLink.

% MODEL: Universal Robotics, UR10, 6DOF, standard_DH



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

function r = mdl_ur10()
    
    deg = pi/180;
    
    % robot length values (metres)
    a = [0, -0.612, -0.5723, 0, 0, 0]';

    d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922]';

    alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]';

    theta = zeros(6,1);
    
    DH = [theta d a alpha];

    mass = [7.1, 12.7, 4.27, 2.000, 2.000, 0.365];

    center_of_mass = [
        0.021, 0, 0.027
        0.38, 0, 0.158
        0.24, 0, 0.068
        0.0, 0.007, 0.018
        0.0, 0.007, 0.018
        0, 0, -0.026  ];    
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    robot = SerialLink(DH, ...
        'name', 'UR10', 'manufacturer', 'Universal Robotics');
    
    % add the mass data, no inertia available
    links = robot.links;
    for i=1:6
        links(i).m = mass(i);
        links(i).r = center_of_mass(i,:);
    end

    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('caller', 'ur10', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2
    end
end