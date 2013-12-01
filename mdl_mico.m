%MDL_MICO Create model of Kinova Mico manipulator
%
%      mdl_mico
%
% Script creates the workspace variable mico which describes the 
% kinematic characteristics of a Kinova Mico manipulator
% using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%
% Reference::
% - "DH Parameters of Mico" Version 1.0.1, August 05, 2013.
%
% Notes::
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, Revolute, mdl_jaco, mdl_puma560, mdl_twolink.


% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function mdl_mico()
    
    deg = pi/180;
    
    % robot length values (metres)
    D1 = 0.2755;
    D2 = 0.2900;
    D3 = 0.1233;
    D4 = 0.0741;
    D5 = 0.0741;
    D6 = 0.1600;
    e2 = 0.0070;
    
    % alternate parameters
    aa = 30*deg;
    ca = cos(aa);
    sa = sin(aa);
    c2a = cos(2*aa);
    s2a = sin(2*aa);
    d4b = D3 + sa/s2a*D4;
    d5b = sa/s2a*D4 + sa/s2a*D5;
    d6b = sa/s2a*D5 + D6;
    
    % DH parameter table
    %     theta d a alpha
    dh = [0 D1   0  pi/2
          0 0    D2 pi
          0 -e2  0  pi/2
          0 -d4b 0  2*aa
          0 -d5b 0  2*aa
          0 -d6b 0  pi];
    
    
    % and build a serial link manipulator
    
    robot = SerialLink(dh, 'name', 'Mico', ...
        'manufacturer', 'Kinova'); 
    
    % place the variables into the global workspace
    if nargout == 0
        assignin('base', 'mico', robot);
        assignin('base', 'qz', [0 0 0 0 0 0]); % zero angles
        assignin('base', 'qr', -[180 270 90 180 180 0]*deg); % ready pose, arm up
    end
end

