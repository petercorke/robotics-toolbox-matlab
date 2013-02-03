%CODEGENERATOR.GENSLBLOCKFRICTION Generate Simulink block for joint friction
%
% cGen.genslblockfriction() generates a robot-specific Simulink block to compute
% the joint friction model.
%
% Notes::
% - Is called by CodeGenerator.genfriction if cGen has active flag genslblock
% - The Simulink blocks are generated and stored in a robot specific block 
%   library cGen.slib in the directory cGen.basepath.
%
% Author::
%  Joern Malzahn
%  2012 RST, Technische Universitaet Dortmund, Germany.
%  http://www.rst.e-technik.tu-dortmund.de
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfriction.

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

function [ F ] = genslblockfriction( CGen )

%% Open or create block library
bdclose('all')                                                              % avoid problems with previously loaded libraries
load_system('simulink');
if ~(exist([CGen.slibpath,simulinkext]) == 2)                                  % Create new block library if none exists
 CGen.createnewblocklibrary;
end
open_system(CGen.slibpath);
set_param(CGen.slib,'lock','off');

[~,qd] = CGen.rob.gencoords;

%% Generate block
CGen.logmsg([datestr(now),'\tGenerating joint friction Embedded Matlab Function Block:']);
symname = 'friction';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if exist(fname,'file')
    tmpStruct = load(fname);
else
    error ('genslblockgfriction:SymbolicsNotFound','Save symbolic expressions to disk first!')
end

blockaddress = [CGen.slib,'/',symname];          % treat intermediate transformations separately
if ~isempty(find_system(CGen.slib,'SearchDepth',1,'Name',symname))                    % Delete previously generated block
    delete_block(blockaddress)
    save_system;
end

symexpr2slblock(blockaddress,tmpStruct.(symname),'vars',{qd});

CGen.logmsg('\t%s\n',' done!');

%% Cleanup
% Arrange blocks
distributeblocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end
