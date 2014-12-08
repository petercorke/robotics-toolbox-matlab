%CODEGENERATOR.FFINDREPLACE Find and replace all occurrences of string in a file.
%
%   CGen.ffindreplace(fName, oText, nText, varargin)
%   FNAME is the absolute or relative path to the file to replace the text in.
%   OTEXT is the text passage to replace.
%   NTEXT is the new text.
%
% Notes::
% The function opens and sweeps the text file FNAME. All occurrences of
% OTEXT are replaced by NTEXT.
%
% Authors::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de) 
%
% See also CodeGenerator.finsertfront.

% Copyright (C) 1993-2012, by Peter I. Corke
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

function [ ] = ffindreplace(CGen, fName, oText, nText, varargin)
 
fid = fopen(fName,'r'); % open the file

% determine number of lines in the file
nLines = getNLines(fid);

% read all lines to cell
oldLines = cell(1,nLines);
newLines = {1,nLines};
for iLines = 1:nLines
    oldLines{iLines} = fgets(fid);
end

% close the file again
fclose(fid);

% replace all occurrences of oText by nText in each line
for iLines = 1:nLines
    newLines{iLines}= strrep(oldLines{iLines}, oText, nText);
end

% rewrite the file
fid = fopen(fName,'w');
for iLines = 1:nLines
   fprintf(fid,'%s',newLines{iLines});
end
fclose(fid);


end

function [nLines] = getNLines(fid)
nLines = 0;
while (~feof(fid))      % go through each line until eof is reached
    fgets(fid);
    nLines = nLines +1; % increment line counter
end
frewind(fid);           % set file indicator back to the beginning of the file
end