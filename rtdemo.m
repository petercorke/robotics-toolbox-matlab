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

% $Log: not supported by cvs2svn $
% Revision 1.3  2002-04-02 12:26:48  pic
% Handle figures better, control echo at end of each script.
% Fix bug in calling ctraj.
%
% Revision 1.2  2002/04/01 11:47:17  pic
% General cleanup of code: help comments, see also, copyright, remnant dh/dyn
% references, clarification of functions.
%
% $Revision: 1.1 $

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

% if demos are not already in the path, add them to the path
if exist('rttrdemo', 'file') == 2
    tbpath = fileparts(which('tr2eul'));
    demopath = fullfile(tbpath, 'demos');
    addpath( demopath );
    disp(['** Adding Robotics Toolbox demos to your Matlab path ' demopath]);
end

echo off
clear all
delete( get(0, 'Children') );

mdl_puma560
while 1
 selection = menu('Robot Toolbox demonstrations', ...
 	'Transformations', ...
 	'Trajectory', ...
 	'Forward kinematics', ...
 	'Animation', ...
 	'Inverse kinematics', ...
 	'Jacobians', ...
 	'Inverse dynamics', ...
 	'Forward dynamics', ...
 	'Exit');

 switch selection
 case 1
 	rttrdemo
 case 2
 	rttgdemo
 case 3
 	rtfkdemo
 case 4
 	rtandemo
 case 5
 	rtikdemo
 case 6
 	rtjademo
 case 7
 	rtidemo
 case 8
 	rtfddemo
 case 9
	delete( get(0, 'Children') );
 	break;
 end
end
