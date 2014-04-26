%RTBDEMO 	Robot toolbox demonstrations
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

echo off
clear all
delete( get(0, 'Children') );

% find the path to the demos
if exist('rtbdemo', 'file') == 2
    tbpath = fileparts(which('rtbdemo'));
    demopath = fullfile(tbpath, 'demos');
end

opts = {'path', demopath};

% uncomment the next line to allow demos to run without needing to press
% enter key at each step
%opts = {'path', demopath, 'delay', 0.5};
    

fprintf('------------------------------------------------------------\n');
fprintf('Many of these demos print tutorial text and MATLAB commmands in the console window.\n');
fprintf('Read the text and press <enter> to move on to the next command\n');
fprintf('At the end of the tutorial::demo you can choose the next one from the graphical menu.\n');
fprintf('------------------------------------------------------------\n');

demos = {
    'General::Rotations', 'rotation';
    'General::Transformations', 'trans';
    'General::JoystickTransform', 'joytest';
    'General::Trajectory', 'traj';
    'Arm::Robots', 'robot';
    'Arm::Animation', 'graphics';
    'Arm::Forward kinematics', 'fkine';
    'Arm::Inverse kinematics', 'ikine';
    'Arm::Jacobians', 'jacob';
    'Arm::Inverse dynamics', 'idyn';
    'Arm::Forward dynamics', 'fdyn';
    'Arm::Symbolic', 'symbolic';
    'Arm::Code generation', 'codegen';
    'Mobile::driving to a pose', 'drivepose';
    'Mobile::quadrotor', 'quadrotor';
    'Mobile::Braitenberg', 'braitnav';
    'Mobile::Bug', 'bugnav';
    'Mobile::D*', 'dstarnav';
    'Mobile::PRM', 'prmnav';
    'Mobile::SLAM', 'slam';
    'Mobile::Particle filter', 'particlefilt';
    'Exit', '';
    };

while true
    selection = menu('Robot Toolbox demonstrations', demos{:,1});
    
    if strcmp(demos{selection,1}, 'Exit')
        % quit now
        delete( get(0, 'Children') );
        break;
    else
        % run the appropriate script
        script = demos{selection,2}
        runscript(script, opts{:})
    end
end
