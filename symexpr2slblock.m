%SYMEXPR2SLBLOCK Create symbolic embedded MATLAB Function block
%
% symexpr2slblock(VARARGIN) creates an Embedded MATLAB Function block 
% from a symbolic expression. The input arguments are just as used with 
% the functions emlBlock or matlabFunctionBlock.
%
% Notes::
% - In Symbolic Toolbox versions prior to V5.7 (2011b) the function to 
%   create Embedded Matlab Function blocks from symbolic expressions is 
%   'emlBlock'.
% - Since V5.7 (2011b) there is another function named 
%   'matlabFunctionBlock' which replaces the old function. 
% - symexpr2slblock is a wrapper around both functions, which 
%   checks for the installed Symbolic Toolbox version and calls the 
%   required function accordingly.
%
% Authors::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also emlBlock, matlabFunctionBlock.


% Copyright (C) 1993-2014, by Peter I. Corke
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

function [] = symexpr2slblock(varargin)
 
% V5.8 (R2012a)
if verLessThan('symbolic','5.7')                                   
    emlBlock(varargin{:});
elseif verLessThan('symbolic','5.11')
    matlabFunctionBlock(varargin{:});
else
    % Work around a bug in matlabFunctionBlock.m
    
    %% Read orignal file
	fid = fopen(which('matlabFunctionBlock.m'),'r');
    funStr = fscanf(fid, '%c',inf);
    fclose(fid);
    
    %% Create Temporary Workaround File
    tmpFName = fullfile(pwd,'tmp_workaround_matlabFunctionBlock.m');
    
    % perform necessary modifications
    funStr = strrep(funStr,...
        'if isa(b,''Stateflow.EMChart'')',...   Buggy expression to be replaced
        'if ~strcmp(class(b),''Stateflow.EMChart'')'); % Working expression from previous version.
    funStr = strrep(funStr,...
        'matlabFunctionBlock',...   Buggy expression to be replaced
        'tmp_workaround_matlabFunctionBlock'); % Working expression from previous version.
    
    % write modified function to temporary file
    fid = fopen(tmpFName, 'w');
    fprintf(fid,'%s\n','% This is a temporary file used as workaround for a bug in matlabFunctionBlock.m (R2013b).');
    fprintf(fid,'%s\n','% It should have been deleted automatically by the Robotics Toolbox. If not, feel free to do it manually.');
    fprintf(fid,'%s',funStr);
    fclose(fid);

    % update function database
    rehash
    
    % perform the actual block generation
    tmp_workaround_matlabFunctionBlock(varargin{:});
    
    % clean up
    delete(tmpFName);
end
