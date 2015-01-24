%STARTUP_RTB Initialize MATLAB paths for Robotics Toolbox
%
% Adds demos, examples to the MATLAB path, and adds also to 
% Java class path.

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
fp = fopen('RELEASE', 'r');
release = fgetl(fp);
fclose(fp);
fprintf('- Robotics Toolbox for Matlab (release %s)\n', release)
tbpath = fileparts(which('Link'));
addpath( fullfile(tbpath, 'demos') );
addpath( fullfile(tbpath, 'examples') );
addpath( fullfile(tbpath, 'mex') );
javaaddpath( fullfile(tbpath, 'DH.jar') );
addpath( fullfile(tbpath, 'interfaces', 'VREP') );
% add the contrib code to the path
a = fullfile(rvcpath, 'contrib', 'arte');
if exist(a, 'dir')
    addpath( a );
    disp([' - ARTE: A ROBOTICS TOOLBOX FOR EDUCATION (' a ')']);
end
p = fullfile(rvcpath, 'contrib', 'pHRIWARE');
if exist(p, 'dir')
    addpath( p );
    disp([' - pHRIWARE (release ',pHRIWARE('ver'),'): ',pHRIWARE('c')]);
end
p = fullfile(rvcpath, 'contrib/paretofront');
if exist(p)
    addpath( p );
    disp([' - paretofront contributed code (' p ')']);
end
[currentversion,status] = urlread('http://www.petercorke.com/RTB/currentversion.php', 'Timeout', 2.0);
if status == 1
    if ~strcmp(release, currentversion)
        fprintf('** Release %s now available\n\n', ...
            currentversion);
    end
end
clear status release currentversion tbpath
disp('Run rtbdemo to explore the toolbox');
