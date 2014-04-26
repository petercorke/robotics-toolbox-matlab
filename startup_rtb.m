%STARTUP_RTB Initialize MATLAB paths for Robotics Toolbox
%
% Adds demos, examples to the MATLAB path, and adds also to 
% Java class path.

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
release = load('RELEASE');
fprintf('- Robotics Toolbox for Matlab (release %.1f)\n', release)
tbpath = fileparts(which('Link'));
addpath( fullfile(tbpath, 'demos') );
addpath( fullfile(tbpath, 'examples') );
addpath( fullfile(tbpath, 'mex') );
javaaddpath( fullfile(tbpath, 'DH.jar') );
%currentversion = urlread('http://www.petercorke.com/RTB/currentversion.php');
currentversion = '0';
currentversion = str2double(currentversion);
%{
if release ~= currentversion
    fprintf('** Release %.1f now available\n\n', ...
        currentversion);
end
%}
% add the contrib code to the path
p = fullfile(rvcpath, 'contrib/arte');
if exist(p)
    addpath( p );
    disp([' - ARTE: A ROBOTICS TOOLBOX FOR EDUCATION (' p ')']);
end
clear release currentversion tbpath
