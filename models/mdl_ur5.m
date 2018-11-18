%MDL_UR5 Create model of Universal Robotics UR5 manipulator
%
% MDL_UR5 is a script that creates the workspace variable ur5 which
% describes the kinematic characteristics of a Universal Robotics UR5 manipulator
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
% See also mdl_ur3, mdl_ur10, mdl_puma560, SerialLink.

% MODEL: Universal Robotics, UR5, 6DOF, standard_DH



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

function r = mdl_ur5()
    
    deg = pi/180;
    
    % robot length values (metres)
    a = [0, -0.42500, -0.39225, 0, 0, 0]';

    d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]';

    alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]';
    
    theta = zeros(6,1);
    
    DH = [theta d a alpha];

    mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897];

    center_of_mass = [
        0,-0.02561, 0.00193
        0.2125, 0, 0.11336
        0.15, 0, 0.0265
        0, -0.0018, 0.01634
        0, -0.0018, 0.01634
        0, 0, -0.001159];
    
    % and build a serial link manipulator
    
    % offsets from the table on page 4, "Mico" angles are the passed joint
    % angles.  "DH Algo" are the result after adding the joint angle offset.

    robot = SerialLink(DH, ...
        'name', 'UR5', 'manufacturer', 'Universal Robotics');
    
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
        assignin('caller', 'ur5', robot);
        assignin('caller', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('caller', 'qr', [180 0 0 0 90 0]*deg); % vertical pose as per Fig 2
    end
end
