%CCODEFUNCTIONSTRING Converts a symbolic expression into a C-code function
%
% [FUNSTR, HDRSTR] = ccodefunctionstring(SYMEXPR, ARGLIST) returns a string
% representing a C-code implementation of a symbolic expression SYMEXPR.
% The C-code implementation has a signature of the form:
%
%         void funname(double[][n_o] out, const double in1, 
%           const double* in2, const double[][n_i] in3);
%
% depending on the number of inputs to the function as well as the
% dimensionality of the inputs (n_i) and the output (n_o).
% The whole C-code implementation is returned in FUNSTR, while HDRSTR
% contains just the signature ending with a semi-colon (for the use in 
% header files).
%
% Options::
% 'funname',name    Specify the name of the generated C-function. If
%                   this optional argument is omitted, the variable name 
%                   of the first input argument is used, if possible.
% 'output',outVar   Defines the identifier of the output variable in the C-function.
% 'vars',varCells   The inputs to the C-code function must be defined as a cell array. The
%                   elements of this cell array contain the symbolic variables required to
%                   compute the output. The elements may be scalars, vectors or matrices
%                   symbolic variables. The C-function prototype will be composed accoringly
%                   as exemplified above.
% 'flag',sig        Specifies if function signature only is generated, default (false).
%
% Example::
%          % Create symbolic variables
%          syms q1 q2 q3
%
%          Q = [q1 q2 q3];
%          % Create symbolic expression
%          myrot = rotz(q3)*roty(q2)*rotx(q1)
%
%          % Generate C-function string
%          [funstr, hdrstr] = ccodefunctionstring(myrot,'output','foo', ...
%          'vars',{Q},'funname','rotate_xyz')
%
% Notes::
% - The function wraps around the built-in Matlab function 'ccode'. It does
%   not check for proper C syntax. You must take care of proper
%   dimensionality of inputs and outputs with respect to your symbolic
%   expression on your own. Otherwise the generated C-function may not
%   compile as desired.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also ccode, matlabFunction.

% Copyright (C) 2012-2018, by Joern Malzahn
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
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

function [funstr hdrstr] = ccodefunctionstring(f,varargin)


% option defaults
opt.funname = inputname(1);
opt.output{1} = zeros(size(f));
opt.outputName{1} = inputname(1);
opt.flag = 0;

if isempty(opt.outputName{1})
    opt.outputName{1} = 'myout';
end
opt.vars = {};

% tb_optparse is not applicable here,
% since handling cell inputs and extracting input variable names is
% required.
% Thus, scan varargin manually:
if mod(nargin,2)==0
    error('CodeGenerator:codefunctionstring:wrongArgumentList',...
        'Wrong number of elements in the argument list.');
end
for iArg = 1:2:nargin-1
    switch lower(varargin{iArg})
        case 'funname'
            opt.funname = varargin{iArg+1};
        case 'output'
            if ~isempty(varargin{iArg+1})
                opt.outputName{1} = varargin{iArg+1};
            end
        case 'vars'
            opt.vars = varargin{iArg+1};
        case 'flag'
            opt.flag = varargin{iArg+1};
        otherwise
            error('ccodefunctionstring:unknownArgument',...
                ['Argument ',inputname(iArg),' unknown.']);
    end
end

nOut = numel(opt.output);
nIn = numel(opt.vars);

%% Function signature
funstr = sprintf('void %s(', opt.funname);
initstr = '';

% outputs
for iOut = 1:nOut
    tmpOutName = opt.outputName{iOut};
    tmpOut = opt.output{iOut};
    
    if ~isscalar(tmpOut);
        funstr = [funstr, sprintf('double %s[][%u]', tmpOutName, size(tmpOut,1) ) ];
        for iRow = 1:size(tmpOut,1)
            for iCol = 1:size(tmpOut,2)
                initstr = sprintf(' %s %s[%u][%u]=0;\n',initstr,tmpOutName,iCol-1,iRow-1);
            end
        end
    else
        funstr = [funstr, sprintf('double %s', tmpOutName ) ];
    end
    
    % separate argument list by commas
    if ( iOut ~= nOut ) || ( nIn > 0 )
        funstr = [funstr,', '];
    end
    
end

% inputs
for iIn = 1:nIn
    tmpInName = ['input',num2str(iIn)];%opt.varsName{iIn};
    tmpIn = opt.vars{iIn};
    
    % treat different dimensionality of input variables
    if isscalar(tmpIn)
        funstr = [funstr, sprintf('const double %s', tmpInName ) ];
    elseif isvector(tmpIn)
        funstr = [funstr, sprintf('const double* %s', tmpInName ) ];
    elseif ismatrix(tmpIn)
        funstr = [funstr, sprintf('const double %s[][%u]', tmpInName, size(tmpIn,2) ) ];
    else
        error('ccodefunctionstring:UnsupportedOutputType', 'Unsupported datatype for %s', tmpOutName)
    end
    
    % separate argument list by commas
    if ( iIn ~= nIn )
        funstr = [funstr,', '];
    end
    
end
funstr = [funstr,sprintf('%s', ')')];

% finalize signature for the use in header files
if nargout > 1
    hdrstr = [funstr,sprintf('%s', ';')];
end
if opt.flag
    return;         %% STOP IF FLAG == TRUE
end

% finalize signature for use in function definition
funstr = [funstr,sprintf('%s', '{')];
funstr = sprintf('%s\n%s',funstr,sprintf('%s', ' ') ); % empty line

%% Function body
% input paramter expansion
for iIn = 1:nIn
    tmpInName = ['input',num2str(iIn)];%opt.varsName{iIn};
    tmpIn = opt.vars{iIn};
    
    % for scalars
    %   -> do nothing
    
    % for vectors
    if ~isscalar(tmpIn) && isvector(tmpIn)
        nEl = numel(tmpIn);
        for iEl = 1:nEl
            funstr = sprintf('%s\n%s',...
                funstr,...
                sprintf('  double %s = %s[%u];', char(tmpIn(iEl)), tmpInName,iEl-1 ));
        end
        
        % for matrices
    elseif ~isscalar(tmpIn) && ~isvector(tmpIn) && ismatrix(tmpIn)
        nRow = size(tmpIn,1);
        nCol = size(tmpIn,2);
        for iRow = 1:nRow
            for iCol = 1:nCol
                
                funstr = sprintf('%s\n%s',...
                    funstr,...
                    sprintf('  double %s%u%u = %s[%u][%u];', char(tmpIn(iRow,iCol)), iRow, iCol, tmpInName{iIn},iRow-1,iCol-1 ));
            end
        end
    end
    
end

funstr = sprintf('%s\n%s',...
    funstr,...
    sprintf('%s', ' ') );
funstr = sprintf('%s\n%s',...
    funstr,...
    sprintf('%s\n\n', initstr) );

% Actual code
% use f.' here, because of column/row indexing in C
codestr = '';
if ~isequal(f, sym(zeros(size(f))))
    eval([opt.outputName{1}, ' = f.''; codestr = ccode(',opt.outputName{1},');'])
end

if isscalar(f)
    % in the case of scalar expressions the resulting ccode always
    % begins with '  t0'. Replace that with the desired name.
    codestr = strrep(codestr,'t0',opt.outputName{1});
end

funstr = sprintf('%s\n%s',...
    funstr,...
    codestr );

funstr = sprintf('%s\n%s',...
    funstr,sprintf('%s', '}') );
funstr = sprintf('%s\n%s',...
    funstr,sprintf('%s', ' ') ); % empty line
