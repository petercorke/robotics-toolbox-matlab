%STARTUP_RTB Initialize MATLAB paths for Robotics Toolbox
%
% Adds demos, data, and examples to the MATLAB path, and adds also to 
% Java class path.
%
% Notes::
% - This sets the paths for the current session only.
% - To make the settings persistent across sessions you can:
%   - Add this script to your MATLAB startup.m script.
%   - After running this script run PATHTOOL and save the path.
%
% See also PATH, ADDPATH, PATHTOOL, JAVAADDPATH.


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


fp = fopen('RELEASE', 'r');
release = fgetl(fp);
fclose(fp);
fprintf('- Robotics Toolbox for MATLAB (release %s)\n', release)
tbpath = fileparts(which('Link'));
addpath( fullfile(tbpath, 'demos') );
addpath( fullfile(tbpath, 'examples') );
addpath( fullfile(tbpath, 'Apps') );
addpath( fullfile(tbpath, 'mex') );
addpath( fullfile(tbpath, 'models') );
addpath( fullfile(tbpath, 'data') );
javaaddpath( fullfile(tbpath, 'java', 'DHFactor.jar') );
addpath( fullfile(tbpath, 'interfaces', 'VREP') );
% add the contrib code to the path
rvcpath = fileparts(tbpath);  % strip one folder off path
p = fullfile(rvcpath, 'contrib');
addpath(p)

p = fullfile(tbpath, 'data', 'ARTE');
disp([' - ARTE contributed code: 3D models for robot manipulators (' p ')']);
p = fullfile(rvcpath, 'contrib', 'pHRIWARE', 'next');
if exist(p, 'dir')
    addpath( p );
    disp([' - pHRIWARE (release ',pHRIWARE('ver'),'): ',pHRIWARE('c')]);
end
p = fullfile(rvcpath, 'contrib/paretofront');
% if exist(p)
%     addpath( p );
%     disp([' - paretofront contributed code (' p ')']);
% end

% [currentversion,status] = urlread('http://www.petercorke.com/RTB/currentversion.php', 'Timeout', 2.0);
% if status == 1
%     if ~strcmp(release, currentversion)
%         fprintf('** Release %s now available\n\n', ...
%             currentversion);
%     end
% end
clear status release currentversion tbpath
