function [] = genslblockfdyn(CGen)
%% GENSLBLOCKFDYN Generates the robot specific real-time capable Simulink block for the robot forward dynamics.
%
%  [] = genslblockfdyn(cGen)
%  [] = cGen.genslblockfdyn
%
%  Inputs::
%       cGen:  a codeGenerator class object
%
%       If cGen has the active flag:
%           - saveresult: the symbolic expressions are saved to
%           disk in the directory specified by cGen.sympath
%
%           - genmfun: ready to use m-functions are generated and
%           provided via a subclass of SerialLink stored in cGen.robjpath
%
%           - genslblock: a Simulink block is generated and stored in a
%           robot specific block library cGen.slib in the directory
%           cGen.basepath
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%
%  See also codeGenerator, genfdyn

% Copyright (C) 1993-2012, by Peter I. Corke
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

%% Open or create block library
bdclose('all')                                                              % avoid problems with previously loaded libraries
load_system('simulink');
if exist([CGen.slibpath,'.mdl']) == 2                                  % Open existing block library if it already exists
    open_system(CGen.slibpath)
else
    new_system(CGen.slib,'Library', 'ErrorIfShadowed');                      % Create new block library if none exists
    open_system(CGen.slib);
    save_system(CGen.slib,CGen.slibpath);
end
set_param(CGen.slib,'lock','off');

%% Generate forward dynamics block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the forward dynamics\n']);
nJoints = CGen.rob.n;

CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
forwDynamicsBlock = [CGen.slib,'/fdyn'];
if ~isempty(find_system(CGen.slib,'Name','fdyn'))                    % Delete previously generated inertia matrix block
    delete_block(forwDynamicsBlock)
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
distributeBlocks(forwDynamicsBlock);
CGen.logmsg('\t%s\n',' done!');

CGen.logmsg([datestr(now),'\tForward dynamics block complete\n']);

%% Cleanup
% Arrange blocks
distributeBlocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end
