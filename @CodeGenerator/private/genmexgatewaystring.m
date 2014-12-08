%GENMEXGATEWAYSTRING Generates a mex gateway function
%
% [FUNSTR] = genmexgatewaystring(SYMEXPR, ARGLIST) returns a string
% representing a C-code implementation of a mex gateway function.
%
% The argumentlist ARGLIST may contain the following property-value pairs
%   PROPERTY, VALUE
% - 'funname', 'name_string'
%   'name_string' is the identifier of the actual computational C-function
%    called by the gateway routine. If this optional argument is omitted, 
%    the identifier 'myfun' is used
%
% - 'vars', {symVar1, symVar2,...}
%    The inputs to the actual computational C-code function called by the 
%    gateway routine must be defined as a cell array. The elements of this 
%    cell array contain the symbolic variables required to compute the 
%    output. The elements may be scalars, vectors or matrices symbolic 
%    variables. 
%
% Example::
% % Create symbolic variables
% syms q1 q2 q3
% 
% Q = [q1 q2 q3];
% % Create symbolic expression
% myrot = rotz(q3)*roty(q2)*rotx(q1)
% 
% % Generate C-function string
% [funstr] = genmexgatewaystring(myrot,'vars',{Q},'funname','rotate_xyz')
%
% Notes::
%
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also ccode, ccodefunctionstring.

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


function [ gwstring ] = genmexgatewaystring(CGen, f, varargin )

%% Read parameters
% option defaults
opt.funname = 'myfun';
opt.output{1} = zeros(size(f));
opt.vars = {};

% tb_optparse is not applicable here,
% since handling cell inputs and extracting input variable names is
% required. Thus, scan varargin manually:
if mod(nargin,2)~=0
    error('CodeGenerator:genmexgatewaystring:wrongArgumentList',...
        'Wrong number of elements in the argument list.');
end

for iArg = 1:2:nargin-2
    switch lower(varargin{iArg})
        case 'funname'
            if ~isempty(varargin{iArg+1})
                opt.funname = varargin{iArg+1};
            end
        case 'vars'
            opt.vars = varargin{iArg+1};
        otherwise
            error('genmexgatewaystring:unknownArgument',...
                ['Argument ',inputname(iArg),' unknown.']);
    end
end

nIn = numel(opt.vars);

%% begin to write the function string
% gwstring = sprintf('\n\n\n');
gwstring = sprintf('\n');

gwstring = [gwstring, sprintf('%s\n','/* The gateway function */')];
gwstring = [gwstring, sprintf('%s\n','void mexFunction( int nlhs, mxArray *plhs[],')];
gwstring = [gwstring, sprintf('%s\n','        int nrhs, const mxArray *prhs[])')];
gwstring = [gwstring, sprintf('%s\n','{')];

%% variable declaration
gwstring = [gwstring, sprintf('\t%s\n','/* variable declarations */')];
% output
gwstring = [gwstring, sprintf('\t%s\n','double* outMatrix;      /* output matrix */')];

% inputs
for iIn = 1:nIn
    tmpInName = ['input',num2str(iIn)];
    tmpIn = opt.vars{iIn};
    gwstring = [gwstring, sprintf('\tdouble* %s;\n', tmpInName ) ];
end

gwstring = [gwstring, sprintf('%s\n',' ')]; % Empty line

%% argument checks
% number of input arguments
gwstring = [gwstring, sprintf('\t%s\n','/* check for proper number of arguments */')];
gwstring = [gwstring, sprintf('\t%s%u%s\n','if(nrhs!=',nIn+1,') {')];
gwstring = [gwstring, sprintf('\t\t%s%s%s\n','mexErrMsgIdAndTxt("',opt.funname,':nrhs",')];
gwstring = [gwstring, sprintf('\t\t%s%u%s\n','                       "',nIn,' inputs required.");')];
gwstring = [gwstring, sprintf('\t%s\n','}')];

% dimensions of input arguments

% number of output arguments
gwstring = [gwstring, sprintf('\t%s%u%s\n','if(nlhs>',1,') {')];
gwstring = [gwstring, sprintf('\t\t%s%s%s\n','mexErrMsgIdAndTxt("',opt.funname,':nlhs",')];
gwstring = [gwstring, sprintf('\t\t%s\n','                       "Only single output allowed.");')];
gwstring = [gwstring, sprintf('\t%s\n','}')];

gwstring = [gwstring, sprintf('%s\n',' ')]; % Empty line

%% pointer initialization for...
% the output
gwstring = [gwstring, sprintf('\t%s\n','/* allocate memory for the output matrix */')];
gwstring = [gwstring, sprintf('\t%s\n',['plhs[0] = mxCreateDoubleMatrix(',num2str(size(f,1)),',',num2str(size(f,2)),', mxREAL);'])];
gwstring = [gwstring, sprintf('\t%s\n','/* get a pointer to the real data in the output matrix */')];
gwstring = [gwstring, sprintf('\t%s\n','outMatrix = mxGetPr(plhs[0]);')];
gwstring = [gwstring, sprintf('%s\n',' ')]; % Empty line

% inputs
gwstring = [gwstring, sprintf('\t%s\n','/* get a pointers to the real data in the input matrices */')];
for iIn = 1:nIn
    tmpInName = ['input',num2str(iIn)];%opt.varsName{iIn};
    tmpIn = opt.vars{iIn};
    
    gwstring = [gwstring, sprintf('\t%s\n',[tmpInName,' = mxGetPr(prhs[',num2str(iIn),']);'])]; % first input argument is the robot object, thus count one-based here
end

gwstring = [gwstring, sprintf('%s\n',' ')]; % Empty line
    
%% call computational routine
gwstring = [gwstring, sprintf('\t%s\n','/* call the computational routine */')];
gwstring = [gwstring, sprintf('\t%s',[opt.funname,'(','outMatrix, '])];

% comma separated list of input arguments
for iIn = 1:nIn
    tmpInName = ['input',num2str(iIn)];

    gwstring = [gwstring, sprintf('%s',tmpInName)];
    
    % separate argument list by commas
    if ( iIn ~= nIn )
        gwstring = [gwstring,', '];
    else
        gwstring = [gwstring,sprintf('%s\n',');')];
    end
end

%% finalize
gwstring = [gwstring, sprintf('%s\n','}')];

