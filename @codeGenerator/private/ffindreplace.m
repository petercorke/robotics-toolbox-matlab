function [ ] = ffindreplace(CGen, fName, oText, nText, varargin)
%% FFINDREPLACE Find and replace all occurrences of string in a file.
%
%   [ ] = ffindreplace(CGen, fName, oText, nText, varargin)
%   [ ] = CGen.ffindreplace(fName, oText, nText, varargin)
%
%  Description::
%    The function opens and sweeps the text file fName. All occurrences of
%    oText are replaced by nText.
%
%  Input::
%    fName:             Name of the file to replace the text in.
%    oText:             Text passage to replace.
%    nText:             New text replacement.
%
%  Authors::
%        Jörn Malzahn   
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de   
%
%  See also finsertfront.


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