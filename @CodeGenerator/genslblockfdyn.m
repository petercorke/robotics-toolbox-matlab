%CODEGENERATOR.GENSLBLOCKFDYN Generate Simulink block for forward dynamics
%
% cGen.genslblockfdyn() generates a robot-specific Simulink block to compute
% forward dynamics.
%
% Notes::
% - Is called by CodeGenerator.genfdyn if cGen has active flag genslblock
% - The generated Simulink block is composed of previously generated blocks
%   for the inertia matrix, coriolis matrix, vector of gravitational load and
%   joint friction vector. The block recombines these components to compute 
%   the forward dynamics.
% - Access to generated function is provided via subclass of SerialLink 
%   whose class definition is stored in cGen.robjpath.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfdyn.

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
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

function [] = genslblockfdyn(CGen)

%% Open or create block library
bdclose('all')                                                              % avoid problems with previously loaded libraries
load_system('simulink');
if ~(exist([CGen.slibpath,simulinkext]) == 2)                                  % Create new block library if none exists
 CGen.createnewblocklibrary;
end
checkexistanceofblocks(CGen);
open_system(CGen.slibpath);
set_param(CGen.slib,'lock','off');

%% Generate forward dynamics block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the forward dynamics\n']);
nJoints = CGen.rob.n;

CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
symname = 'fdyn';
forwDynamicsBlock = [CGen.slib,'/',symname];
if ~isempty(find_system(CGen.slib,'SearchDepth',1,'Name',symname))                    % Delete previously generated block
    delete_block(forwDynamicsBlock)
    save_system;
end
CGen.logmsg('\t%s\n',' done!');

% add required blocks
CGen.logmsg([datestr(now),'\t\t... adding Simulink blocks for all components']);
add_block('built-in/SubSystem',forwDynamicsBlock);
add_block([CGen.slib,'/invinertia'],[forwDynamicsBlock,'/invinertia']);
add_block([CGen.slib,'/coriolis'],[forwDynamicsBlock,'/coriolis']);
add_block([CGen.slib,'/gravload'],[forwDynamicsBlock,'/gravload']);
add_block([CGen.slib,'/friction'],[forwDynamicsBlock,'/friction']);
add_block('Simulink/Sinks/Out1',[forwDynamicsBlock,'/q']);
add_block('Simulink/Sinks/Out1',[forwDynamicsBlock,'/qDot']);
add_block('Simulink/Sinks/Out1',[forwDynamicsBlock,'/qDDot']);
add_block('Simulink/Sources/In1',[forwDynamicsBlock,'/tau']);
add_block('built-in/Sum',[forwDynamicsBlock,'/Sum'],'inputs','|+---');
add_block('built-in/Product',[forwDynamicsBlock,'/invInertiaMultiply'],'multiplication','Matrix(*)');
add_block('built-in/Product',[forwDynamicsBlock,'/coriolisMultiply'],'multiplication','Matrix(*)');
add_block('built-in/Integrator',[forwDynamicsBlock,'/Integrator1']);
add_block('built-in/Integrator',[forwDynamicsBlock,'/Integrator2']);
CGen.logmsg('\t%s\n',' done!');

% connect simulink blocks
CGen.logmsg([datestr(now),'\t\t... wiring']);
add_line(forwDynamicsBlock,'tau/1','Sum/1');
add_line(forwDynamicsBlock,'invInertiaMultiply/1','qDDot/1');
add_line(forwDynamicsBlock,'invInertiaMultiply/1','Integrator1/1');
add_line(forwDynamicsBlock,'Integrator1/1','Integrator2/1');
add_line(forwDynamicsBlock,'Integrator1/1','qDot/1');
add_line(forwDynamicsBlock,'Integrator2/1','q/1');
add_line(forwDynamicsBlock,'Integrator1/1','coriolis/2');
add_line(forwDynamicsBlock,'Integrator1/1','coriolisMultiply/2');
add_line(forwDynamicsBlock,'Integrator1/1','friction/1');
add_line(forwDynamicsBlock,'Integrator2/1','coriolis/1');
add_line(forwDynamicsBlock,'Integrator2/1','gravload/1');
add_line(forwDynamicsBlock,'Integrator2/1','invinertia/1');
add_line(forwDynamicsBlock,'invinertia/1','invInertiaMultiply/1');
add_line(forwDynamicsBlock,'Sum/1','invInertiaMultiply/2');
add_line(forwDynamicsBlock,'coriolisMultiply/1','Sum/2');
add_line(forwDynamicsBlock,'gravload/1','Sum/3');
add_line(forwDynamicsBlock,'friction/1','Sum/4');
add_line(forwDynamicsBlock,'coriolis/1','coriolisMultiply/1');
distributeblocks(forwDynamicsBlock);
CGen.logmsg('\t%s\n',' done!');

CGen.logmsg([datestr(now),'\tForward dynamics block complete\n']);

%% Cleanup
% Arrange blocks
distributeblocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end

function [] = checkexistanceofblocks(CGen)
open_system(CGen.slibpath);
if isempty(find_system(CGen.slib,'SearchDepth',1,'Name','inertia')) || isempty(find_system(CGen.slib,'SearchDepth',1,'Name','invinertia'))
    CGen.logmsg('\t\t%s\n','Inertia block not found! Generating:');
    CGen.genslblockinertia;
    open_system(CGen.slibpath);
end

if isempty(find_system(CGen.slib,'SearchDepth',1,'Name','coriolis'))
    CGen.logmsg('\t\t%s\n','Coriolis block not found! Generating:');
    CGen.genslblockcoriolis;
    open_system(CGen.slibpath);
end

if isempty(find_system(CGen.slib,'SearchDepth',1,'Name','gravload'))
    CGen.logmsg('\t\t%s\n','Gravload block not found! Generating:');
    CGen.genslblockgravload;
    open_system(CGen.slibpath);
end

if isempty(find_system(CGen.slib,'SearchDepth',1,'Name','friction'))
    CGen.logmsg('\t\t%s\n','Friction block not found! Generating:');
    CGen.genslblockfriction;
    open_system(CGen.slibpath);
end

end
