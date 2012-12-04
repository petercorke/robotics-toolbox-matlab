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