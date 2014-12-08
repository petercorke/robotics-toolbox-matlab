%MEXFUNCTION Converts a symbolic expression into a MEX-function
%
% [] = cGen.mexfunction(SYMEXPR, ARGLIST) translates a symbolic expression
% into C-code and joins it with a MEX gateway routine. The resulting C-file
% is ready to be compiled using the matlab built-in mex command.
%
% The argumentlist ARGLIST may contain the following property-value pairs
%   PROPERTY, VALUE
% - 'funname', 'name_string'
%   'name_string' is the actual identifier of the obtained C-function. 
%   Default: 'myfun'
%
% - 'funfilename', 'file_name_string'
%   'file_name_string' is the name of the generated C-file including 
%   relativepath or absolute path information.
%   Default: 'myfunfilename'
%
% - 'output', 'output_name'
%   Defines the identifier of the output variable in the C-function.
%   Default: 'out'
%
% - 'vars', {symVar1, symVar2,...}
%    The inputs to the C-code function must be defined as a cell array. The
%    elements of this cell array contain the symbolic variables required to
%    compute the output. The elements may be scalars, vectors or matrices
%    symbolic variables. The C-function prototype will be composed accordingly
%    as exemplified above.
%
% - 'header', hStruct
%   Struct containing header information. See constructheaderstring
%   Default: []
%
% Example::
%
%
% Notes::
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also mex, ccode, matlabFunction.

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

function [] = mexfunction(CGen, f, varargin )

%% Read parameters
% option defaults
opt.funname = 'myfun';
opt.vars = {};
opt.output = 'out';
opt.funfilename = 'myfunfilename';
opt.hStruct = [];

% tb_optparse is not applicable here,
% since handling cell inputs and extracting input variable names is
% required. Thus, scan varargin manually:
if mod(nargin,2)~=0
    error('CodeGenerator:mexfunction:wrongArgumentList',...
        'Wrong number of elements in the argument list.');
end

for iArg = 1:2:nargin-2
    switch lower(varargin{iArg})
        case 'funname'
            if ~isempty(varargin{iArg+1})
                opt.funname = varargin{iArg+1};
            end
        case 'funfilename'
            if ~isempty(varargin{iArg+1})
                opt.funfilename = varargin{iArg+1};
            end
        case 'output'
            if ~isempty(varargin{iArg+1})
                opt.outputName{1} = varargin{iArg+1};
            end
        case 'vars'
            opt.vars = varargin{iArg+1};
        case 'header'
            opt.hStruct = varargin{iArg+1};
        otherwise
            error('genmexgatewaystring:unknownArgument',...
                ['Argument ',inputname(iArg),' unknown.']);
    end
end

%% Create Copyright Note
cprNote = CGen.generatecopyrightnote;
cprNote = regexprep(cprNote, '%', '//');

%% Create Compilation Command
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');
if CGen.verbose
    mexCompCmnd = ['mex ',opt.funfilename, ' ',fullfile(srcDir,[opt.funname,'.c']),' -I',hdrDir, ' -v -outdir ',CGen.robjpath];   
else
    mexCompCmnd = ['mex ',opt.funfilename, ' ',fullfile(srcDir,[opt.funname,'.c']),' -I',hdrDir,' -outdir ',CGen.robjpath];
end

%% Generate C code
fid = fopen(opt.funfilename,'w+');

% Add compilation note
fprintf(fid,'/* %s\n',[upper(opt.funname) ,' - This file contains auto generated C-code for a MATLAB MEX function.']);    
fprintf(fid,'// %s\n',['For details on how to use the complied MEX function see the documentation provided in ',opt.funname,'.m']);    
fprintf(fid,'// %s\n//\n',['The compiled MEX function replaces this .m-function with identical usage but substantial execution speedup.']);    
fprintf(fid,'// %s\n//\n',['For compilation of this C-code using MATLAB please run:']);    
fprintf(fid,'// \t\t%s\n//\n',['''',mexCompCmnd,'''']);    
fprintf(fid,'// %s\n',['Make sure you have a C-compiler installed and your MATLAB MEX environment readily configured.']);
fprintf(fid,'// %s\n//\n',['Type ''doc mex'' for additional help.']); 

% Insert Copyright Note 
fprintf(fid,'// %s\n','__Copyright Note__:');
fprintf(fid,'%s */\n',cprNote);

% Includes
fprintf(fid,'%s\n%s\n\n',...
    '#include "mex.h"',...
    ['#include "',[opt.funname,'.h'],'"']);

% Generate the mex gateway routine
funstr = CGen.genmexgatewaystring(f,'funname',opt.funname, 'vars',opt.vars);
fprintf(fid,'%s',sprintf(funstr));

fclose(fid);

%% Compile the MEX file
if CGen.compilemex
    eval(mexCompCmnd)
end