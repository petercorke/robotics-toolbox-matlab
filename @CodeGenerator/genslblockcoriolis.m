%CODEGENERATOR.GENSLBLOCKCORIOLIS Generate Simulink block for Coriolis matrix
%
% cGen.genslblockcoriolis() generates a robot-specific Simulink block to compute
% Coriolis/centripetal matrix.
%
% Notes::
% - Is called by CodeGenerator.gencoriolis if cGen has active flag genslblock
% - The Coriolis matrix is stored row by row to avoid memory issues.
% - The Simulink block recombines the the individual blocks for each row.
% - Access to generated function is provided via subclass of SerialLink 
%   whose class definition is stored in cGen.robjpath.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.gencoriolis.

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
function [ ] = genslblockcoriolis( CGen )

%% Open or create block library
bdclose('all')                                                              % avoid problems with previously loaded libraries
load_system('simulink');
if ~(exist([CGen.slibpath,simulinkext]) == 2)                                  % Create new block library if none exists
 CGen.createnewblocklibrary;
end
open_system(CGen.slibpath);
set_param(CGen.slib,'lock','off');

[q,qd] = CGen.rob.gencoords;

%% Generate Coriolis Block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the robot Coriolis matrix\n']);
nJoints = CGen.rob.n;
symname = 'coriolis';

CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
CoriolisBlock = [CGen.slib,'/',symname];
if ~isempty(find_system(CGen.slib,'SearchDepth',1,'Name',symname))                    % Delete previously generated block
    delete_block(CoriolisBlock)
    save_system;
end

% Subsystem in which individual rows are concatenated
add_block('built-in/SubSystem',CoriolisBlock);                               % Add new inertia matrix block
add_block('Simulink/Math Operations/Matrix Concatenate'...
    , [CoriolisBlock,'/coriolis']...
    , 'NumInputs',num2str(nJoints)...
    , 'ConcatenateDimension','1');
add_block('Simulink/Sinks/Out1',[CoriolisBlock,'/out']);
add_block('Simulink/Sources/In1',[CoriolisBlock,'/q']);
add_block('Simulink/Sources/In1',[CoriolisBlock,'/qd']);
add_line(CoriolisBlock,'coriolis/1','out/1');
CGen.logmsg('\t%s\n',' done!');

for kJoints = 1:nJoints
    CGen.logmsg([datestr(now),'\t\t... Embedded Matlab Function Block for joint ',num2str(kJoints),': ']);
    
    % Generate Embedded Matlab Function block for each row
    symname = ['coriolis_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if exist(fname,'file')
        tmpStruct = load(fname);
    else
        error ('genslblockcoriolis:SymbolicsNotFound','Save symbolic expressions to disk first!')
    end
    
    blockaddress = [CoriolisBlock,'/',symname];
    if doesblockexist(CGen.slib,symname)
        delete_block(blockaddress);
        save_system;
    end
    
    CGen.logmsg('%s',' block creation');
    symexpr2slblock(blockaddress,tmpStruct.(symname),'vars',{q,qd});
    
    
    % connect output
    CGen.logmsg('%s',', output wiring');
    if ( verLessThan('matlab','7.11.0.584') ) && ( isequal(tmpStruct.(symname),zeros(1,nJoints)) )
        % There is a bug in earlier Matlab versions. If the symbolic
        % vector is a zero vector, then the Simulink Embedded Matlab
        % Function block outputs a scalar zero. We need to concatenate
        % a row vector of zeros here, which we have to construct on our
        % own.
        add_block('Simulink/Math Operations/Matrix Concatenate'...              % Use a matrix concatenation block ...
            , [CoriolisBlock,'/DimCorrection',num2str(kJoints)]...               % ... named with the current row number ...
            , 'NumInputs',num2str(nJoints),'ConcatenateDimension','2');         % ... intended to concatenate zero values for each joint ...
        % ... columnwise. This will circumvent the bug.
        
        for iJoints = 1:nJoints                                                 % Connect signal lines from the created block (which outputs
            add_line(CoriolisBlock...                                            % a scalar zero in this case) with the bugfix block.
                , [symname,'/1']...
                , ['DimCorrection',num2str(kJoints),'/', num2str(iJoints)]);
        end
        
        add_line(CoriolisBlock,['DimCorrection',num2str(kJoints)...              % Connect the fixed row with other rows.
            , '/1'],['coriolis/', num2str(kJoints)]);
        
    else
        add_line(CoriolisBlock,[symname,'/1']...          % In case that no bug occurs, we can just connect the rows.
            , ['coriolis/', num2str(kJoints)]);
    end
    
    % Vector generalized joint values
    add_line(CoriolisBlock,'q/1',[symname,'/1']);
    % Vector generalized joint velocities
    add_line(CoriolisBlock,'qd/1',[symname,'/2']);
    
    CGen.logmsg('\t%s\n','row complete!');
end
addterms(CoriolisBlock); % Add terminators where needed
distributeblocks(CoriolisBlock);
CGen.logmsg([datestr(now),'\tCoriolis matrix block complete\n']);


%% Cleanup
% Arrange blocks
distributeblocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end
