%CODEGENERATOR.GENMFUNINVDYN Generate M-functions for inverse dynamics
% 
% cGen.genmfuninvdyn() generates a robot-specific M-function to compute
% inverse dynamics.
%
% Notes::
% - Is called by CodeGenerator.geninvdyn if cGen has active flag genmMfun
% - The generated M-function is composed of previously generated M-functions
%   for the inertia matrix, coriolis matrix, vector of gravitational load and 
%   joint friction vector.  This function recombines these components to 
%   compute the forward dynamics.
% - Access to generated function is provided via subclass of SerialLink 
%   whose class definition is stored in cGen.robjpath.
%
% Author::
%  Joern Malzahn
%  2012 RST, Technische Universitaet Dortmund, Germany.
%  http://www.rst.e-technik.tu-dortmund.de
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninvdyn.

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

function [  ] = genmfuninvdyn( CGen )

%% Does robot class exist?
if ~exist(fullfile(CGen.robjpath,[CGen.getrobfname,'.m']),'file')
    CGen.logmsg([datestr(now),'\tCreating ',CGen.getrobfname,' m-constructor ']);
    CGen.createmconstructor;
    CGen.logmsg('\t%s\n',' done!');
end
checkexistanceofmfunctions(CGen);

%%
CGen.logmsg([datestr(now),'\tGenerating inverse dynamics m-function']);

funfilename = fullfile(CGen.robjpath,'invdyn.m');
hStruct = createHeaderStruct(CGen.rob,funfilename);

fid = fopen(funfilename,'w+');

fprintf(fid, '%s\n', ['function tau = invdyn(rob,q,qd,qdd)']);                 % Function definition
fprintf(fid, '%s\n',constructheaderstring(CGen,hStruct));                   % Header

fprintf(fid, '%s \n', 'tau = zeros(length(q),1);');                        % Code

funcCall = ['tau = rob.inertia(q)*qdd(:) + ',...
    'rob.coriolis(q,qd)*qd(:) + ',...
    'rob.gravload(q).'' - ', ...
    'rob.friction(qd).'';'];
fprintf(fid, '%s \n', funcCall);


fclose(fid);

CGen.logmsg('\t%s\n',' done!');
end

function [] = checkexistanceofmfunctions(CGen)

if ~(exist(fullfile(CGen.robjpath,'inertia.m'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Inertia m-function not found! Generating:');
    CGen.genmfuninertia;
end

if ~(exist(fullfile(CGen.robjpath,'coriolis.m'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Coriolis m-function not found! Generating:');
    CGen.genmfuncoriolis;
end

if ~(exist(fullfile(CGen.robjpath,'gravload.m'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Gravload m-function not found! Generating:');
    CGen.genmfungravload;
end

if ~(exist(fullfile(CGen.robjpath,'friction.m'),'file') == 2)
    CGen.logmsg('\t\t%s\n','Friction m-function not found! Generating:');
    CGen.genmfunfriction;
end

end

%% Definition of the header contents for each generated file
function hStruct = createHeaderStruct(rob,fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['Inverse dynamics for the',rob.name,' arm.'];
hStruct.calls = {['tau = ',hStruct.funName,'(rob,q,qd,qdd)'],...
    ['tau = rob.',hStruct.funName,'(q,qd,qdd)']};
hStruct.detailedDescription = {'Given a full set of joint variables and their first and second order',...
    'temporal derivatives this function computes the joint space',...
    'torques needed to perform the particular motion.'};
hStruct.inputs = { ['rob: robot object of ', rob.name, ' specific class'],...
                   ['q:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     coordinates',...
                   ['qd:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     velocities', ...
                   ['qdd:  ',int2str(rob.n),'-element vector of generalized'],...
                   '     accelerations',...
                   'Angles have to be given in radians!'};
hStruct.outputs = {['tau:  [',int2str(rob.n),'x1] vector of joint forces/torques.']};
hStruct.references = {'1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar',...
    '2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano',...
    '3) Introduction to Robotics, Mechanics and Control - Craig',...
    '4) Modeling, Identification & Control of Robots - Khalil & Dombre'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn',...
    '2012 RST, Technische Universitaet Dortmund, Germany',...
     'http://www.rst.e-technik.tu-dortmund.de'};
hStruct.seeAlso = {'fdyn'};
end
