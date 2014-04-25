%CODEGENERATOR.GENCCODEFKINE Generate C-code for the forward kinematics
%
% cGen.genccodefkine() generates a robot-specific C-function to compute
% forward kinematics.
%
% Notes::
% - Is called by CodeGenerator.genfkine if cGen has active flag genccode or
%   genmex
% - The generated .c and .h files are wirtten to the directory specified in
%   the ccodepath property of the CodeGenerator object.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfkine, CodeGenerator.genmexfkine.

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

function [] = genccodefkine(CGen)

%% Check for existance symbolic expressions
% Load symbolics
symname = 'fkine';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if exist(fname,'file')
    tmpStruct = load(fname);
else
    error ('genmfunfkine:SymbolicsNotFound','Save symbolic expressions to disk first!')
end

%% Forward kinematics up to tool center point
CGen.logmsg([datestr(now),'\tGenerating forward kinematics c-code up to the end-effector frame: ']);

%% Prerequesites
% check for existance of C-code directories
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');
if ~exist(srcDir,'dir')
    mkdir(srcDir);
end
if ~exist(hdrDir,'dir')
    mkdir(hdrDir);
end

funname = [CGen.getrobfname,'_',symname];
funfilename = [funname,'.c'];
hfilename = [funname,'.h'];

Q = CGen.rob.gencoords;


% Convert symbolic expression into C-code
[funstr hstring] = ccodefunctionstring(tmpStruct.(symname),...
    'funname',funname,...
    'vars',{Q},'output','T');

% Create the function description header
hStruct = createHeaderStructFkine(CGen.rob,symname); % create header
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

CGen.logmsg('\t%s\n',' done!');


%% Individual joint forward kinematics
CGen.logmsg([datestr(now),'\tGenerating forward kinematics m-function up to joint: ']);
for iJoints=1:CGen.rob.n
    CGen.logmsg(' %i ',iJoints);
    
    % Load symbolics
    symname = ['T0_',num2str(iJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    tmpStruct = struct;
    tmpStruct = load(fname);
    
    funname = [CGen.getrobfname,'_',symname];
    funfilename = [funname,'.c'];
    hfilename = [funname,'.h'];
    Q = CGen.rob.gencoords;
    
        % Convert symbolic expression into C-code
    [funstr hstring] = ccodefunctionstring(tmpStruct.(symname),...
        'funname',funname,...
        'vars',{Q},'output','T');
        
    % Create the function description header
    hStruct = createHeaderStruct(CGen.rob,iJoints,funname); % create header
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
    
    % Description header
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

end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStruct(rob,curBody,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['C version of the forward kinematics for the ',rob.name,' arm up to frame ',int2str(curBody),' of ',int2str(rob.n),'.'];
hStruct.detailedDescription = {['Given a set of joint variables up to joint number ',int2str(curBody),' the function'],...
    'computes the pose belonging to that joint with respect to the base frame.'};
hStruct.inputs = { 'input1 Vector of generalized coordinates. Angles have to be given in radians!'};
hStruct.outputs = {'T [4x4] Homogenous transformation matrix relating the pose of the tool for the given joint values to the base frame.'};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!<BR>',...
    'Code generator written by:<BR>',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)<BR>'};
hStruct.seeAlso = {rob.name};
end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStructFkine(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['C version of the forward kinematics solution including tool transformation for the ',rob.name,' arm.'];
hStruct.detailedDescription = {['Given a full set of joint variables the function'],...
    'computes the pose belonging to that joint with respect to the base frame.'};
hStruct.inputs = { 'input1 Vector of generalized coordinates. Angles have to be given in radians!'};
hStruct.outputs = {'T [4x4] Homogenous transformation matrix relating the pose of the tool for the given joint values to the base frame.'};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!<BR>',...
    'Code generator written by:<BR>',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)<BR>'};
hStruct.seeAlso = {'jacob0'};
end

