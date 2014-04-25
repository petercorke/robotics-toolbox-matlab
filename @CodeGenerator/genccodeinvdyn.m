%CODEGENERATOR.GENCCODEINVDYN Generate C-code for inverse dynamics
% 
% cGen.genccodeinvdyn() generates a robot-specific C-code to compute the
% inverse dynamics.
%
% Notes::
% - Is called by CodeGenerator.geninvdyn if cGen has active flag genccode or
%   genmex.
% - The .c and .h files are generated in the directory specified
%   by the ccodepath property of the CodeGenerator object.
% - The resulting C-function is composed of previously generated C-functions
%   for the inertia matrix, coriolis matrix, vector of gravitational load and 
%   joint friction vector. This function recombines these components to 
%   compute the inverse dynamics.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninvdyn, CodeGenerator.genccodefdyn.

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
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [  ] = genccodeinvdyn( CGen )

checkexistanceofcfunctions(CGen);
[Q, QD, QDD] = CGen.rob.gencoords;
nJoints = CGen.rob.n;

% Check for existance of C-code directories
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');
if ~exist(srcDir,'dir')
    mkdir(srcDir);
end
if ~exist(hdrDir,'dir')
    mkdir(hdrDir);
end

symname = 'invdyn';
outputname = 'TAU';

funname = [CGen.getrobfname,'_',symname];
funfilename = [funname,'.c'];
hfilename = [funname,'.h'];

%%
CGen.logmsg([datestr(now),'\tGenerating inverse dynamics C-code']);

% Convert symbolic expression into C-code
dummy = sym(zeros(nJoints,1));
[funstr hstring] = ccodefunctionstring(dummy,...
    'funname',funname,...
    'vars',{Q, QD, QDD},'output',outputname,...
    'flag',1);

% Create the function description header
hStruct = createHeaderStruct(CGen.rob,funname); % create header
hStruct.calls = hstring;
hFString = CGen.constructheaderstringc(hStruct);

%% Generate C implementation file
fid = fopen(fullfile(srcDir,funfilename),'w+');

% Includes
fprintf(fid,'%s\n\n',...
    ['#include "', hfilename,'"']);

% Function
fprintf(fid,'%s{\n\n',funstr);


% Allocate memory
fprintf(fid,'%s\n','/* declare variables */');
for iJoints = 1:nJoints
	fprintf(fid,'%s\n',['double inertia_row',num2str(iJoints),'[',num2str(nJoints),'][1];']);
end

for iJoints = 1:nJoints
    fprintf(fid,'%s\n',['double coriolis_row',num2str(iJoints),'[',num2str(nJoints),'][1];']);
end

fprintf(fid,'%s\n',['double gravload[',num2str(nJoints),'][1];']);

fprintf(fid,'%s\n',['double friction[',num2str(nJoints),'][1];']);

fprintf(fid,'%s\n',' '); % empty line
fprintf(fid,'%s\n','/* call the computational routines */');
fprintf(fid,'%s\n',[CGen.getrobfname,'_','gravload(gravload, input1);']);
fprintf(fid,'%s\n',[CGen.getrobfname,'_','friction(friction, input2);']);

fprintf(fid,'%s\n','/* rowwise routines */');
for iJoints = 1:nJoints
    fprintf(fid,'%s\n',[CGen.getrobfname,'_','inertia_row_',num2str(iJoints),'(inertia_row',num2str(iJoints),', input1);']);
end

for iJoints = 1:nJoints
    fprintf(fid,'%s\n',[CGen.getrobfname,'_','coriolis_row_',num2str(iJoints),'(coriolis_row',num2str(iJoints),', input1, input2);']);
end

fprintf(fid,'%s\n',' '); % empty line
fprintf(fid,'%s\n','/* fill output vector */');

for iJoints = 1:nJoints
    fprintf(fid,'%s\n',[outputname,'[0][',num2str(iJoints-1),'] = dotprod(inertia_row',num2str(iJoints),', input3, ',num2str(nJoints),') /* inertia */']);
    fprintf(fid,'\t%s\n',[' + dotprod(coriolis_row',num2str(iJoints),', input2, ',num2str(nJoints),') /* coriolis */']);
    fprintf(fid,'\t%s\n',[' + gravload[',num2str(iJoints-1),'][0]']);
    fprintf(fid,'\t%s\n',[' - friction[',num2str(iJoints-1),'][0];']);
end
fprintf(fid,'%s\n',' '); % empty line

fprintf(fid,'%s\n','}');

fclose(fid);

%% Generate C header file
fid = fopen(fullfile(hdrDir,hfilename),'w+');

% Function description header
fprintf(fid,'%s\n\n',hFString);

% Include guard
fprintf(fid,'%s\n%s\n\n',...
    ['#ifndef ', upper([funname,'_h'])],...
    ['#define ', upper([funname,'_h'])]);

% Includes
fprintf(fid,'%s\n%s\n%s\n%s\n%s\n\n',...
    '#include "dotprod.h"',...
    ['#include "',CGen.getrobfname,'_inertia.h"'],...
    ['#include "',CGen.getrobfname,'_coriolis.h"'],...
    ['#include "',CGen.getrobfname,'_gravload.h"'],...
    ['#include "',CGen.getrobfname,'_friction.h"']);


fprintf(fid,'%s\n\n',hstring);

% Include guard
fprintf(fid,'%s\n',...
    ['#endif /*', upper([funname,'_h */'])]);

fclose(fid);

CGen.logmsg('\t%s\n',' done!');

end

function [] = checkexistanceofcfunctions(CGen)

if ~(exist(fullfile(CGen.ccodepath,'src','inertia.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','inertia.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Inertia C-code not found or not complete! Generating:');
    CGen.genccodeinertia;
end

if ~(exist(fullfile(CGen.ccodepath,'src','coriolis.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'coriolis','inertia.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Coriolis C-code not found or not complete! Generating:');
    CGen.genccodecoriolis;
end

if ~(exist(fullfile(CGen.ccodepath,'src','gravload.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','gravload.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Gravload C-code not found or not complete! Generating:');
    CGen.genccodegravload;
end

if ~(exist(fullfile(CGen.ccodepath,'src','friction.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','friction.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Friction C-code not found or not complete! Generating:');
    CGen.genccodefriction;
end

if ~(exist(fullfile(CGen.ccodepath,'src','dotprod.c'),'file') == 2) || ~(exist(fullfile(CGen.ccodepath,'include','dotprod.h'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Dot product C-code not found or not complete! Generating:');
    CGen.gendotprodc;
end

end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStruct(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['C-implementation of the inverse dynamics for the ',rob.name,' arm.'];
hStruct.detailedDescription = {'Given a full set of joint variables and their first and second order',...
    'temporal derivatives this function computes the joint space',...
    'torques needed to perform the particular motion. Angles have to be given in radians!'};
hStruct.inputs = { ['input1:  ',int2str(rob.n),'-element vector of generalized coordinates'],...
                   ['input2:  ',int2str(rob.n),'-element vector of generalized velocities,'],...
                   ['input3:  ',int2str(rob.n),'-element vector of generalized accelerations.']};
hStruct.outputs = {['TAU:  [',int2str(rob.n),'x1] vector of joint forces/torques.']};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'fdyn'};
end
