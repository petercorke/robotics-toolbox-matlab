function [ output_args ] = genslblockinvdyn( CGen )
%% GENSLBLOCKINVDYN Generates the robot specific Embedded Matlab Function Block for the inverse dynamics.
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%

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
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
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

%% Generate Inertia Block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the inverse robot dynamics:\n']);
nJoints = CGen.rob.n;

CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
InvDynBlock = [CGen.slib,'/invdyn'];
if ~isempty(find_system(CGen.slib,'Name','invdyn'))                    % Delete previously generated inertia matrix block
    delete_block(InvDynBlock)
end
CGen.logmsg('\t%s\n',' done!');

% Subsystem in which individual rows are concatenated
CGen.logmsg([datestr(now),'\t\t... adding Embedded Matlab Function Blocks for all components ']);
add_block('built-in/SubSystem',InvDynBlock);                               % Add new inertia matrix block
add_block([CGen.slib,'/inertia'],[InvDynBlock,'/inertia']);
add_block([CGen.slib,'/coriolis'],[InvDynBlock,'/coriolis']);
add_block([CGen.slib,'/gravload'],[InvDynBlock,'/gravload']);
add_block([CGen.slib,'/friction'],[InvDynBlock,'/friction'])
add_block('Simulink/Sources/In1',[InvDynBlock,'/q']);
add_block('Simulink/Sources/In1',[InvDynBlock,'/qd']);
add_block('Simulink/Sources/In1',[InvDynBlock,'/qdd']);
add_block('Simulink/Sinks/Out1',[InvDynBlock,'/tau']);
add_block('built-in/Product',[InvDynBlock,'/inertiaTorque'],'multiplication','Matrix(*)');
add_block('built-in/Product',[InvDynBlock,'/coriolisTorque'],'multiplication','Matrix(*)');
add_block('built-in/Sum',[InvDynBlock,'/Sum'],'inputs','++++');
CGen.logmsg('\t%s\n',' done!');

% Connect individual Simulink blocks
CGen.logmsg([datestr(now),'\t\t... wiring']);
add_line(InvDynBlock,'q/1','inertia/1');
add_line(InvDynBlock,'q/1','coriolis/1');
add_line(InvDynBlock,'q/1','gravload/1');
add_line(InvDynBlock,'qd/1','coriolis/2');
add_line(InvDynBlock,'qd/1','friction/1');
add_line(InvDynBlock,'inertia/1','inertiaTorque/1');
add_line(InvDynBlock,'qdd/1','inertiaTorque/2');
add_line(InvDynBlock,'coriolis/1','coriolisTorque/1');
add_line(InvDynBlock,'qd/1','coriolisTorque/2');
add_line(InvDynBlock,'inertiaTorque/1','Sum/1');
add_line(InvDynBlock,'coriolisTorque/1','Sum/2');
add_line(InvDynBlock,'gravload/1','Sum/3');
add_line(InvDynBlock,'friction/1','Sum/4');
add_line(InvDynBlock,'Sum/1','tau/1');
CGen.logmsg('\t%s\n',' done!');

CGen.logmsg([datestr(now),'\tInverse dynamics block complete.\n']);

%% Cleanup
% Arrange blocks
distributeBlocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end