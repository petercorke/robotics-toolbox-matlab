%STARTUP_RTB Initialize MATLAB paths for Robotics Toolbox
%
% Adds demos, examples to the MATLAB path, and adds also to 
% Java class path.
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
