function genslblockgravload(CGen)
%% GENSLBLOCKGRAVLOAD Generates the robot specific Embedded Matlab Function Blocks for the gravitational load forces/torques.
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

%% Generate Block
CGen.logmsg([datestr(now),'\tGenerating Embedded Matlab Function Block for the vector of gravitational load forces/torques ']);
symname = 'gravload';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if exist(fname,'file')
    tmpStruct = load(fname);
else
    error ('genslblockgravload:SymbolicsNotFound','Save symbolic expressions to disk first!')
end

blockaddress = [CGen.slib,'/',symname];
if doesblockexist(CGen.slib,symname)
    delete_block(blockaddress);
    save_system;
end

symexpr2slblock(blockaddress,tmpStruct.(symname));

CGen.logmsg('\t%s\n',' done!');

%% Cleanup
% Arrange blocks
distributeBlocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

end