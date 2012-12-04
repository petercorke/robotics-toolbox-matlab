function [] = createmconstructor(CGen)
%% CREATEMCONSTRUCTOR Creates the constructor of the specialized robot class collecting the generated m-function code.
%
%  Authors::
%        Jörn Malzahn   
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de     
%
%  See also genfkine, genmfunfkine.
%
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