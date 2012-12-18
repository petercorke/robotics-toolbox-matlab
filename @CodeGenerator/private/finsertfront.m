function [ ] = finsertfront(CGen, fName,nText)
%% FINSERTFRONT Insert a string at the beginning of a textfile.
% =========================================================================
%
%   [ ] = finsertfront( CGen, fName,nText)
%   [ ] = CGen.finsertfront( fName,nText)
%
%  Description::
%    MatLab ships with functions for reading, overwriting and appending text
%    to files. This function is used to insert text at the beginning of a
%    text file.
%
%  Input::
%       fName:     Full or relative path to the text file.
%       nText:     String containing the text to be inserted at the beginning 
%                  of the file.
%
%  Authors::
%        Jörn Malzahn   
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de     
%
%  See also ffindreplace.
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
 
%% analyse the file
fid = fopen(fName,'r'); % open the file
% determine number of lines in the file
nLines = getNLines(fid);
% read all lines to cell
oldLines = cell(1,nLines);
for iLines = 1:nLines
    oldLines{iLines} = fgets(fid);
end
% close the file again
fclose(fid);

%% rewrite the file
fid = fopen(fName,'w');
fprintf(fid,'%s',nText);

for iLines = 1:nLines
   fprintf(fid,'%s',oldLines{iLines});
end
fclose(fid);


end

%% Determine the number of lines in a file
function [nLines] = getNLines(fid)
nLines = 0;
while (~feof(fid))      % go through each line until eof is reached
    fgets(fid);
    nLines = nLines +1; % increment line counter
end
frewind(fid);           % set file indicator back to the beginning of the file
end