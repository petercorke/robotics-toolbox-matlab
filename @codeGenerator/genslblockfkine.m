function genslblockfkine(CGen)
%% GENSLBLOCKFKINE Generates Embedded Matlab Function blocks from the symbolic robot specific forward kinematics expressions.
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%

%% Preparation
% Output to logfile?
if ~isempty(CGen.logfile)
    logfid = fopen(CGen.logfile,'a');
else
    logfid = [];
end

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

%% Forward kinematics up to tool center point
multidfprintf([CGen.verbose, logfid],...
    [datestr(now),'\tGenerating forward kinematics Embedded Matlab Function Block up to the end-effector frame: ']);
symname = 'fkine';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if exist(fname,'file')
    tmpStruct = load(fname);
else
    error ('genSLBlockFkine:SymbolicsNotFound','Save symbolic expressions to disk first!')
end

blockaddress = [CGen.slib,'/',symname];          % treat intermediate transformations separately
if doesblockexist(CGen.slib,symname)
    delete_block(blockaddress);
    save_system;
end

symexpr2slblock(blockaddress,tmpStruct.(symname));

multidfprintf([CGen.verbose, logfid],'\t%s\n',' done!');

%% Individual joint forward kinematics
multidfprintf([CGen.verbose, logfid],...
    [datestr(now),'\tGenerating forward kinematics Embedded Matlab Function Block up to joint: ']);
for iJoints=1:CGen.rob.n
    
    multidfprintf([CGen.verbose, logfid],' %i ',iJoints);
    symname = ['T0_',num2str(iJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    tmpStruct = struct;
    tmpStruct = load(fname);
    
    funFileName = fullfile(CGen.robjpath,[symname,'.m']);
    q = CGen.rob.gencoords;
    
    
    blockaddress = [CGen.slib,'/',symname];          % treat intermediate transformations separately
    if doesblockexist(CGen.slib,symname)
        delete_block(blockaddress);
        save_system;
    end
    
    symexpr2slblock(blockaddress,tmpStruct.(symname));
    
end
multidfprintf([CGen.verbose, logfid],'\t%s\n',' done!');

%% Cleanup
% Arrange blocks
distributeBlocks(CGen.slib);

% Lock, save and close library
set_param(CGen.slib,'lock','on');
save_system(CGen.slib,CGen.slibpath);
close_system(CGen.slib);

% Logfile to close?
if ~isempty(CGen.logfile)
    logfid = fclose(logfid);
end

end