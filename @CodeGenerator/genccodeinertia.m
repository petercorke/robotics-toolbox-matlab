%CODEGENERATOR.GENCCODEINERTIA Generate C-function for robot inertia matrix
%
% cGen.genccodeinertia() generates robot-specific C-functions to compute
% the robot inertia matrix.
%
% Notes::
% - Is called by CodeGenerator.geninertia if cGen has active flag genccode or
%   genmex.
% - The generated .c and .h files are generated in the directory specified
%   by the ccodepath property of the CodeGenerator object.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninertia, CodeGenerator.genmexinertia.

% Copyright (C) 2012-2014, by Joern Malzahn
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
% along with RTB. If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [] = genccodeinertia(CGen)

%%
CGen.logmsg([datestr(now),'\tGenerating C-code for the robot inertia matrix row' ]);


% check for existance of C-code directories
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');
if ~exist(srcDir,'dir')
    mkdir(srcDir);
end
if ~exist(hdrDir,'dir')
    mkdir(hdrDir);
end


Q = CGen.rob.gencoords;
nJoints = CGen.rob.n;

%% Individual inertia matrix rows
for kJoints = 1:nJoints
    CGen.logmsg(' %s ',num2str(kJoints));
    symname = ['inertia_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if exist(fname,'file')
        tmpStruct = load(fname);
    else
        error ('genmfuninertia:SymbolicsNotFound','Save symbolic expressions to disk first!')
    end
    
    funname = [CGen.getrobfname,'_',symname];
    funfilename = [funname,'.c'];
    hfilename = [funname,'.h'];
    
    % Convert symbolic expression into C-code
    [funstr hstring] = ccodefunctionstring(tmpStruct.(symname),...
        'funname',funname,...
        'vars',{Q},'output',['I_row_',num2str(kJoints)]);
    
    % Create the function description header
    hStruct = createHeaderStructRow(CGen.rob,kJoints,funname); % create header
    hStruct.calls = hstring;
    hFString = CGen.constructheaderstringc(hStruct);
   
    %% Generate C implementation file
    fid = fopen(fullfile(srcDir,funfilename),'w+');

    % Includes
    fprintf(fid,'%s\n\n',...
        ['#include "', hfilename,'"']);
    
    % Function
    fprintf(fid,'%s\n\n',funstr);
    fclose(fid);
    
    %% Generate C header file
    fid = fopen(fullfile(hdrDir,hfilename),'w+');
    
    % Header
    fprintf(fid,'%s\n\n',hFString);
    
    % Include guard
    fprintf(fid,'%s\n%s\n\n',...
        ['#ifndef ', upper([funname,'_h'])],...
        ['#define ', upper([funname,'_h'])]);
    
    % Includes
    fprintf(fid,'%s\n\n',...
        '#include "math.h"');
    
    % Function prototype
    fprintf(fid,'%s\n\n',hstring);
    
    % Include guard
    fprintf(fid,'%s\n',...
        ['#endif /*', upper([funname,'_h */'])]);
    
    fclose(fid);
    
end
CGen.logmsg('\t%s\n',' done!');


%% Full inertia matrix
CGen.logmsg([datestr(now),'\tGenerating full inertia matrix C-code']);

symname = 'inertia';

funname = [CGen.getrobfname,'_',symname];
funfilename = [funname,'.c'];
hfilename = [funname,'.h'];
outname = 'I';

% Generate function prototype
[hstring] = ccodefunctionstring(sym(zeros(nJoints)),...
    'funname',funname,...
    'vars',{Q},'output',outname,'flag',1);

% Create the function description header
hStruct = createHeaderStructFull(CGen.rob,funname); % create header
hStruct.calls = hstring;
hFString = CGen.constructheaderstringc(hStruct);


%% Generate C implementation file
fid = fopen(fullfile(srcDir,funfilename),'w+');

% Includes
fprintf(fid,'%s\n\n',...
    ['#include "', hfilename,'"']);

fprintf(fid,'%s{\n\n',hstring);

fprintf(fid,'\t%s\n','/* allocate memory for individual rows */');
for kJoints = 1:nJoints
    fprintf(fid,'\tdouble row%d[%d][1];\n',kJoints,nJoints);
end
fprintf(fid,'%s\n',' '); % empty line

fprintf(fid,'\t%s\n','/* call the row routines */');
for kJoints = 1:nJoints
    fprintf(fid,'\t%s_inertia_row_%d(row%d, input1);\n',CGen.getrobfname,kJoints,kJoints);
end
fprintf(fid,'%s\n',' '); % empty line\n

% Copy results into output matrix
for iJoints = 1:nJoints
    for kJoints = 1:nJoints
        fprintf(fid,'\t%s[%d][%d] = row%d[%d][0];\n',outname,kJoints-1,iJoints-1,iJoints,kJoints-1);
    end
    fprintf(fid,'%s\n',' '); % empty line\n
end

fprintf(fid,'%s\n','}');

fclose(fid);

%% Generate C header file
fid = fopen(fullfile(hdrDir,hfilename),'w+');

% Header
fprintf(fid,'%s\n\n',hFString);

% Include guard
fprintf(fid,'%s\n%s\n\n',...
    ['#ifndef ', upper([funname,'_h'])],...
    ['#define ', upper([funname,'_h'])]);

% Includes
fprintf(fid,'%s\n\n',...
    '#include "math.h"');
for kJoints = 1:nJoints
    rowstring = [CGen.getrobfname,'_inertia_row_',num2str(kJoints)];
    fprintf(fid,'%s\n',...
        ['#include "',rowstring,'.h"']);
end
fprintf(fid,'%s\n',' '); % empty line

% Function prototype
fprintf(fid,'%s;\n\n',hstring);

% Include guard
fprintf(fid,'%s\n',...
    ['#endif /*', upper([funname,'_h */'])]);

fclose(fid);

CGen.logmsg('\t%s\n',' done!');
end


%% Definition of the header contents for each generated file
function hStruct = createHeaderStructRow(rob,curJointIdx,fName)
[~,hStruct.funName] = fileparts(fName);
hStruct.shortDescription = ['Computation of the robot specific inertia matrix row for corresponding to joint ', num2str(curJointIdx), ' of ',num2str(rob.n),'.'];
hStruct.detailedDescription = {'Given a full set of joint variables this function computes the',...
    ['inertia matrix row number ', num2str(curJointIdx),' of ',num2str(rob.n),' for ',rob.name,'. Angles have to be given in radians!']};
hStruct.inputs = {['input1:  ',int2str(rob.n),'-element vector of generalized coordinates.']};
hStruct.outputs = {['I_row_',int2str(curJointIdx),':  [1x',int2str(rob.n),'] output row of the robot inertia matrix.']};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'coriolis'};
end

function hStruct = createHeaderStructFull(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['Inertia matrix for the ',rob.name,' arm.'];
hStruct.detailedDescription = {'Given a full set of joint variables the function computes the',...
    'inertia Matrix of the robot. Angles have to be given in radians!'};
hStruct.inputs = {['input1:  ',int2str(rob.n),'-element vector of generalized coordinates.']};
hStruct.outputs = {['I:  [',int2str(rob.n),'x',int2str(rob.n),']output inertia matrix.']};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'coriolis'};
end