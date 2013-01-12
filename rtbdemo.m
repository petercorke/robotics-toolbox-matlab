%RTDEMO 	Robot toolbox demonstrations
%
% Displays popup menu of toolbox demonstration scripts that illustrate:
%   - homogeneous transformations
%   - trajectories
%   - forward kinematics
%   - inverse kinematics
%   - robot animation
%   - inverse dynamics
%   - forward dynamics
%
% Notes::
% - The scripts require the user to periodically hit <Enter> in order to move
%   through the explanation.
% - Set PAUSE OFF if you want the scripts to run completely automatically.

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

echo off
clear all
delete( get(0, 'Children') );

% find the path to the demos
if exist('rtbdemo', 'file') == 2
    tbpath = fileparts(which('tr2eul'));
    demopath = fullfile(tbpath, 'demos');
end

fprintf('------------------------------------------------------------\n');
fprintf('Many of these demos print tutorial text and MATLAB commmands in the console window.\n');
fprintf('Read the text and press <enter> to move on to the next command\n');
fprintf('At the end of the tutorial/demo you can choose the next one from the graphical menu.\n');
fprintf('------------------------------------------------------------\n');

while 1
    selection = menu('Robot Toolbox demonstrations', ...
        'General/Rotations', ...
        'General/Transformations', ...
        'General/Trajectory', ...
        'Arm/Robots', ...
        'Arm/Animation', ...
        'Arm/Forward kinematics', ...
        'Arm/Inverse kinematics', ...
        'Arm/Jacobians', ...
        'Arm/Inverse dynamics', ...
        'Arm/Forward dynamics', ...
        'Arm/Symbolic', ...
        'Arm/Code generation', ...
        'Mobile/driving to a pose', ...
        'Mobile/quadrotor', ...
        'Mobile/Braitenberg', ...
        'Mobile/Bug', ...
        'Mobile/D*', ...
        'Mobile/PRM', ...
        'Mobile/SLAM', ...
        'Mobile/Particle filter', ...
        'Exit');
    
    opts = {'path', demopath};
        opts = {'path', demopath, 'delay', 0.5};

    
    switch selection
        case 1
            runscript('rotation', opts{:})
        case 2
            runscript('trans', opts{:})
        case 3
            runscript('traj', opts{:})
        case 4
            runscript('robot', opts{:})
        case 5
            runscript('fkine', opts{:})
        case 6
            runscript('graphics', opts{:})
        case 7
            runscript('ikine', opts{:})
        case 8
            runscript('jacob', opts{:})
        case 9
            runscript('idyn', opts{:})
        case 10
            runscript('fdyn', opts{:})
        case 11
            runscript('symbolic', opts{:})
        case 12
            runscript('codegen', opts{:})
        case 13
            runscript('drivepose', opts{:})
        case 14
            runscript('quadrotor', opts{:})
        case 15
            runscript('braitnav', opts{:})
        case 16
            runscript('bugnav', opts{:})
        case 17
            runscript('dstarnav', opts{:})
        case 18
            runscript('prmnav', opts{:})
        case 19
            runscript('slam', opts{:})
        case 20
            runscript('particlefilt', opts{:})
        case 21
            delete( get(0, 'Children') );
            break;
    end
end
