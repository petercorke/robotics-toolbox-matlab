%CODEGENERATOR.CREATEMCONSTRUCTOR Creates the constructor of the specialized robot class collecting the generated m-function code.
%
% cGen.createmconstructor()
%
% Authors::
%  Joern Malzahn   
%  2012 RST, Technische Universitaet Dortmund, Germany
%  http://www.rst.e-technik.tu-dortmund.de     
%
% See also CodeGenerator.genfkine, CodeGenerator.genmfunfkine.

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

function [] = createmconstructor(CGen)
 
objdir = CGen.robjpath;
robname = CGen.getrobfname;

if ~exist(objdir,'dir')
    mkdir(objdir)
end

sr = CGen.rob;
save(fullfile(objdir,['mat',robname]), 'sr');

fid = fopen(fullfile(objdir,[robname,'.m']),'w+');

fprintf(fid,'%s\n',['classdef ',robname,' < SerialLink']);
fprintf(fid,'%s\n',' ');
fprintf(fid,'\t%s\n', 'properties');
fprintf(fid,'\t%s\n','end');
fprintf(fid,'%s\n',' ');
fprintf(fid,'\t%s\n','methods');
fprintf(fid,'\t\t%s\n',['function ro = ',robname,'()']);
fprintf(fid,'\t\t\t%s\n',['objdir = which(''',robname,''');']);
fprintf(fid,'\t\t\t%s\n','idx = find(objdir == filesep,2,''last'');');
fprintf(fid,'\t\t\t%s\n','objdir = objdir(1:idx(1));');
fprintf(fid,'\t\t\t%s\n',' ');
fprintf(fid,'\t\t\t%s\n',['tmp = load(fullfile(objdir,',['''@',robname],''',''mat',robname,'.mat''));']);
fprintf(fid,'\t\t\t%s\n',' ');
fprintf(fid,'\t\t\t%s\n','ro = ro@SerialLink(tmp.sr);');
fprintf(fid,'\t\t\t%s\n', ' ');
fprintf(fid,'\t\t\t%s\n',' ');           
fprintf(fid,'\t\t%s\n','end');
fprintf(fid,'\t%s\n','end');
fprintf(fid,'\t%s\n',' ');
fprintf(fid,'%s\n','end');

fclose(fid);