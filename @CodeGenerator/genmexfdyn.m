%CODEGENERATOR.GENMEXFDYN Generate C-MEX-function for forward dynamics
%
% cGen.genmexfdyn() generates a robot-specific MEX-function to compute
% the forward dynamics.
% 
% Notes::
% - Is called by CodeGenerator.genfdyn if cGen has active flag genmex
% - The MEX file uses the .c and .h files generated in the directory 
%   specified by the ccodepath property of the CodeGenerator object.
% - Access to generated function is provided via subclass of SerialLink
%   whose class definition is stored in cGen.robjpath.
% - You will need a C compiler to use the generated MEX-functions. See the 
%   MATLAB documentation on how to setup the compiler in MATLAB. 
%   Nevertheless the basic C-MEX-code as such may be generated without a
%   compiler. In this case switch the cGen flag compilemex to false.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfdyn, CodeGenerator.genmexinvdyn.

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

function [] = genmexfdyn(CGen)

CGen.logmsg([datestr(now),'\tGenerating inverse dynamics MEX-function: ']);

mexfunname = 'accel';
mexcfilename = fullfile(CGen.robjpath,[mexfunname,'.c']);

cfunname = [CGen.getrobfname,'_',mexfunname];
cfilename = [cfunname '.c'];
hfilename = [cfunname '.h'];



[Q, QD] = CGen.rob.gencoords;
[tau] = CGen.rob.genforces;

% Function description header
hStruct = createHeaderStruct(CGen.rob,mexfunname); 
hFString = CGen.constructheaderstringc(hStruct);

fid = fopen(mexcfilename,'w+');

% Insert description header
fprintf(fid,'%s\n',hFString);

% Includes
fprintf(fid,'%s\n%s\n\n',...
    '#include "mex.h"',...
    ['#include "',hfilename,'"']);

dummy = sym(zeros(CGen.rob.n,1));
% Generate the mex gateway routine
funstr = CGen.genmexgatewaystring(dummy,'funname',cfunname, 'vars',{Q, QD, tau});
fprintf(fid,'%s',sprintf(funstr));

fclose(fid);

%% Compile the MEX file
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');

cfilelist = fullfile(srcDir,cfilename);
for kJoints = 1:CGen.rob.n
    cfilelist = [cfilelist, ' ',fullfile(srcDir,[CGen.getrobfname,'_inertia_row_',num2str(kJoints),'.c'])];
end
for kJoints = 1:CGen.rob.n
    cfilelist = [cfilelist, ' ',fullfile(srcDir,[CGen.getrobfname,'_coriolis_row_',num2str(kJoints),'.c'])];
end

cfilelist = [cfilelist, ' ', fullfile(srcDir,[CGen.getrobfname,'_inertia.c'])];
cfilelist = [cfilelist, ' ', fullfile(srcDir,[CGen.getrobfname,'_coriolis.c'])];
cfilelist = [cfilelist, ' ', fullfile(srcDir,[CGen.getrobfname,'_gravload.c'])];
cfilelist = [cfilelist, ' ', fullfile(srcDir,[CGen.getrobfname,'_friction.c'])];
cfilelist = [cfilelist, ' ', fullfile(srcDir,'matvecprod.c')];
cfilelist = [cfilelist, ' ', fullfile(srcDir,'gaussjordan.c')];

if CGen.verbose
    eval(['mex ',mexcfilename, ' ',cfilelist,' -I',hdrDir, ' -v -outdir ',CGen.robjpath]);   
else
    eval(['mex ',mexcfilename, ' ',cfilelist,' -I',hdrDir,' -outdir ',CGen.robjpath]);
end

CGen.logmsg('\t%s\n',' done!');

end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStruct(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.calls = '';
hStruct.shortDescription = ['C-implementation of the forward dynamics for the ',rob.name,' arm.'];
hStruct.detailedDescription = {'Given a full set of joint angles and velocities',...
    'this function computes the joint space accelerations effected by the generalized forces. Angles have to be given in radians!'};
hStruct.inputs = { ['input1:  ',int2str(rob.n),'-element vector of generalized coordinates'],...
    ['input2:  ',int2str(rob.n),'-element vector of generalized velocities'],...
    ['input3:  [',int2str(rob.n),'x1] vector of generalized forces.']};
hStruct.outputs = {['QDD:  ',int2str(rob.n),'-element output vector of generalized accelerations.']};
hStruct.references = {'Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    'Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    'Introduction to Robotics, Mechanics and Control - Craig',...
    'Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'invdyn'};
end