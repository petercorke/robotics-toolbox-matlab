%CODEGENERATOR.GENSLBLOCKINERTIA Generate Simulink block for inertia matrix
%
% cGen.genslbgenslblockinertia() generates a robot-specific Simulink block to compute
% robot inertia matrix.
%
% Notes::
% - Is called by CodeGenerator.geninertia if cGen has active flag genslblock
% - The Inertia matrix is stored row by row to avoid memory issues.
% - The Simulink block recombines the the individual blocks for each row.
% - The Simulink blocks are generated and stored in a robot specific block 
%   library cGen.slib in the directory cGen.basepath.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninertia.

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

function genslblockinertia(CGen)

%% Open or create block library
bdclose('all')                                                              % avoid problems with previously loaded libraries
load_system('simulink');
if ~(exist([CGen.slibpath,simulinkext]) == 2)                                  % Create new block library if none exists
 CGen.createnewblocklibrary;
end
open_system(CGen.slibpath);
set_param(CGen.slib,'lock','off');

q = CGen.rob.gencoords;

%% Generate Inertia Block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the robot inertia matrix\n']);
nJoints = CGen.rob.n;

CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
symname = 'inertia';
InertiaBlock = [CGen.slib,'/',symname];
    if ~isempty(find_system(CGen.slib,'SearchDepth',1,'Name',symname))                    % Delete previously generated inertia matrix block
        delete_block(InertiaBlock);
        save_system;
    end
% Subsystem in which individual rows are concatenated
add_block('built-in/SubSystem',InertiaBlock);                               % Add new inertia matrix block
add_block('Simulink/Math Operations/Matrix Concatenate'...
    , [InertiaBlock,'/inertia']...
    , 'NumInputs',num2str(nJoints)...
    , 'ConcatenateDimension','1');
add_block('Simulink/Sinks/Out1',[InertiaBlock,'/out']);
add_block('Simulink/Sources/In1',[InertiaBlock,'/q']);
add_line(InertiaBlock,'inertia/1','out/1');
CGen.logmsg('\t%s\n',' done!');

for kJoints = 1:nJoints
    CGen.logmsg([datestr(now),'\t\t... Embedded Matlab Function Block for joint ',num2str(kJoints),': ']);
    
    % Generate Embedded Matlab Function block for each row
    symname = ['inertia_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if exist(fname,'file')
        tmpStruct = load(fname);
    else
        error ('genslblockinertia:SymbolicsNotFound','Save symbolic expressions to disk first!')
    end
    
    blockaddress = [InertiaBlock,'/',symname];
    if doesblockexist(CGen.slib,symname)
        delete_block(blockaddress);
        save_system;
    end
    
    CGen.logmsg('%s',' block creation');
    symexpr2slblock(blockaddress,tmpStruct.(symname),'vars',{q});
    
    
    % connect output
    CGen.logmsg('%s',', output wiring');
    if ( verLessThan('matlab','7.11.0.584') ) && ( isequal(tmpStruct.(symname),zeros(1,nJoints)) )
        % There is a bug in earlier Matlab versions. If the symbolic
        % vector is a zero vector, then the Simulink Embedded Matlab
        % Function block outputs a scalar zero. We need to concatenate
        % a row vector of zeros here, which we have to construct on our
        % own.
        add_block('Simulink/Math Operations/Matrix Concatenate'...              % Use a matrix concatenation block ...
            , [InertiaBlock,'/DimCorrection',num2str(kJoints)]...               % ... named with the current row number ...
            , 'NumInputs',num2str(nJoints),'ConcatenateDimension','2');         % ... intended to concatenate zero values for each joint ...
        % ... columnwise. This will circumvent the bug.
        
        for iJoints = 1:nJoints                                                 % Connect signal lines from the created block (which outputs
            add_line(InertiaBlock...                                            % a scalar zero in this case) with the bugfix block.
                , [symname,'/1']...
                , ['DimCorrection',num2str(kJoints),'/', num2str(iJoints)]);
        end
        
        add_line(InertiaBlock,['DimCorrection',num2str(kJoints)...              % Connect the fixed row with other rows.
            , '/1'],['inertia/', num2str(kJoints)]);
        
    else
        add_line(InertiaBlock,[symname,'/1']...          % In case that no bug occurs, we can just connect the rows.
            , ['inertia/', num2str(kJoints)]);
    end
    
    % Connect inputs
    CGen.logmsg('%s',', input wiring');
    add_line(InertiaBlock,'q/1',[symname,'/1']);
    CGen.logmsg('\t%s\n','row complete!');
end
addterms(InertiaBlock); % Add terminators where needed
distributeblocks(InertiaBlock);
CGen.logmsg([datestr(now),'\tInertia matrix block complete\n']);


%% Built inverse Inertia matrix block
CGen.logmsg([datestr(now),'\tGenerating Simulink Block for the inverse robot inertia matrix\n']);
CGen.logmsg([datestr(now),'\t\t... enclosing subsystem ']);
% block address
symname = 'invinertia';
invInertiaBlock = [CGen.slib,'/',symname];
% remove any existing blocks
    if ~isempty(find_system(CGen.slib,'SearchDepth',1,'Name',symname))                    % Delete previously generated block
        delete_block(invInertiaBlock);
        save_system;
    end
add_block('built-in/SubSystem',invInertiaBlock);
CGen.logmsg('\t%s\n',' done!');

% matrix inversion
CGen.logmsg([datestr(now),'\t\t... matrix inversion block ']);
add_block('Simulink/Math Operations/Product',[invInertiaBlock,'/inverse']); % Use a product block...
set_param([invInertiaBlock,'/inverse'],'Inputs','/');                       % ... with single input '/'...
set_param([invInertiaBlock,'/inverse'],'Multiplication','Matrix(*)');       % ... and matrix multiplication
CGen.logmsg('\t%s\n',' done!');

% wire the input and output
CGen.logmsg([datestr(now),'\t\t... input and output ']);
add_block(InertiaBlock,[invInertiaBlock,'/inertiaMatrix']);
add_block('Simulink/Sources/In1',[invInertiaBlock,'/q']);
add_block('Simulink/Sinks/Out1',[invInertiaBlock,'/out']);


% wire the blocks among each other
CGen.logmsg('and internal wiring');
add_line(invInertiaBlock,'q/1','inertiaMatrix/1');
add_line(invInertiaBlock,'inertiaMatrix/1','inverse/1');
add_line(invInertiaBlock,'inverse/1','out/1');
CGen.logmsg('\t%s\n',' done!');

% add terminators where necessary
addterms(invInertiaBlock);
distributeblocks(invInertiaBlock);
CGen.logmsg([datestr(now),'\tInverse inertia matrix block complete.\n']);

%% Cleanup
% Arrange blocks
distributeblocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end
