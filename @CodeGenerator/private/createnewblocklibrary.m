function [] = createnewblocklibrary(CGen)
%CodeGenerator.createnewblocklibrary
%
% Creates a new Simulink library file along with its slblocks.m.
%
% Authors::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also SerialLink, Link.

% Copyright (C) 1993-2012, by Peter I. Corke
% Copyright (C) 2012-2013, by Joern Malzahn
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
%
% http://www.petercorke.com
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

warning off;
CGen.rmpath;
warning on;

%% Create Simulink library file
new_system(CGen.slib,'Library', 'ErrorIfShadowed'); 
open_system(CGen.slib);
save_system(CGen.slib,CGen.slibpath);

%% Create slblock.m

fid = fopen(fullfile(CGen.basepath,'slblocks.m'),'w+');

fprintf(fid,'%s\n',['% This file is the slblocks file for the ', CGen.getrobfname,' Simulink library.']);
fprintf(fid,'%s\n',['% It has been automatically generated using The Robotics Toolbox for Matlab (RTB).']);
fprintf(fid,'%s\n',' ');
fprintf(fid,'%s\n\n',CGen.generatecopyrightnote);

fprintf(fid,'%s\n','function blkStruct = slblocks');
fprintf(fid,'%s\n',' ');
fprintf(fid,'%s\n', ['Browser.Library = ''',CGen.slib,''';']);
fprintf(fid,'%s\n', ['Browser.Name = ''',CGen.getrobfname,''';']);
fprintf(fid,'%s\n', ['Browser.IsFlat = 1;']);
fprintf(fid,'%s\n',' ');

fprintf(fid,'%s\n', ['blkStruct.Name = [''',CGen.getrobfname,''', sprintf(''\n''), ''Library''];']);
fprintf(fid,'%s\n', ['blkStruct.OpenFcn= ''',CGen.slib,''';']);
fprintf(fid,'%s\n', ['blkStruct.MaskDisplay = ''''',';']);
fprintf(fid,'%s\n', ['blkStruct.Bworser = Browser;']);
fprintf(fid,'%s\n',' ');

fclose(fid);
