%CODEGENERATOR.GENGAUSSJORDANC Generates a Gauss-Jordan C-implementation.
%
% cGen.gengaussjordanc generates a .h and a .c file in the directory
% specified by ccodepath.
%
% Notes::
% - Is called by genfdyn if cGen has active flag genmex or genccode.
%
% Authors::
%  Joern Malzahn   (joern.malzahn@tu-dortmund.de) 
%
% See also CodeGenerator, CodeGenerator.gendotprodc.

% Copyright (C) 1993-2014, by Peter I. Corke
% Copyright (C) 2014, by Joern Malzahn
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
% You should have received a copy of the GNU Lesser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [] =  gengaussjordanc(CGen)

funname = 'gaussjordan';
funfilename = [funname,'.c'];
hfilename = [funname,'.h'];
srcDir = fullfile(CGen.ccodepath,'src');
hdrDir = fullfile(CGen.ccodepath,'include');

% Check for already existing gauss-jordan C-files
if exist(fullfile(srcDir,funfilename),'file') && exist(fullfile(srcDir,funfilename),'file')
    return;
end

% Check for existance of C-code directories
if ~exist(srcDir,'dir')
    mkdir(srcDir);
end
if ~exist(hdrDir,'dir')
    mkdir(hdrDir);
end

% Create the function description header
hStruct = createHeaderStruct(funname); % create header
if ~isempty(hStruct)
    hFString = CGen.constructheaderstringc(hStruct);
end

%% Generate C implementation file
fid = fopen(fullfile(srcDir,funfilename),'w+');

% Includes
fprintf(fid,'%s\n\n',...
    ['#include "', hfilename,'"']);

% Start actual function implementation
fprintf(fid,'%s\n',['void ',funname,'(const double* inMatrix, double* outMatrix, int dim){']);

fprintf(fid,'%s\n',' '); % empty line

% variable declarations
fprintf(fid,'\t%s\n','int iRow, iCol, diagIndex;');
fprintf(fid,'\t%s\n','double diagFactor, tmpFactor;');
fprintf(fid,'\t%s\n','double* inMatrixCopy = (double*) malloc(dim*dim*sizeof(double));');

fprintf(fid,'%s\n',' '); % empty line

% input initialization
fprintf(fid,'\t%s\n','/* make deep copy of input matrix */');
fprintf(fid,'\t%s\n','for(iRow = 0; iRow < dim; iRow++ ){');
fprintf(fid,'\t\t%s\n','for (iCol = 0; iCol < dim; iCol++){');
fprintf(fid,'\t\t\t%s\n','inMatrixCopy[dim*iCol+iRow] = inMatrix[dim*iCol+iRow];');
fprintf(fid,'\t\t%s\n','}');

fprintf(fid,'\t%s\n','}');
fprintf(fid,'\t%s\n','/* Initialize output matrix as identity matrix. */');
fprintf(fid,'\t%s\n','for (iRow = 0; iRow < dim; iRow++ ){');
fprintf(fid,'\t\t%s\n','for (iCol = 0; iCol < dim; iCol++ ){');
fprintf(fid,'\t\t\t%s\n','if (iCol == iRow){');
fprintf(fid,'\t\t\t\t%s\n','outMatrix[dim*iCol+iRow] = 1;');
fprintf(fid,'\t\t\t%s\n','}');
fprintf(fid,'\t\t\t%s\n','else{');
fprintf(fid,'\t\t\t\t%s\n','outMatrix[dim*iCol+iRow] = 0;');
fprintf(fid,'\t\t\t%s\n','}');
fprintf(fid,'\t\t%s\n','}');
fprintf(fid,'\t%s\n','}');

fprintf(fid,'%s\n',' '); % empty line

% actual elimination
fprintf(fid,'\t%s\n','for (diagIndex = 0; diagIndex < dim; diagIndex++ )');
fprintf(fid,'\t%s\n','{');
fprintf(fid,'\t\t%s\n','/* determine diagonal factor */');
fprintf(fid,'\t\t%s\n','diagFactor = inMatrixCopy[dim*diagIndex+diagIndex];');

fprintf(fid,'%s\n',' '); % empty line
        
fprintf(fid,'\t\t%s\n','/* divide column entries by diagonal factor */');
fprintf(fid,'\t\t%s\n','for (iCol = 0; iCol < dim; iCol++){');
fprintf(fid,'\t\t\t%s\n','inMatrixCopy[dim*iCol+diagIndex] /= diagFactor;');
fprintf(fid,'\t\t\t%s\n','outMatrix[dim*iCol+diagIndex] /= diagFactor;');
fprintf(fid,'\t\t%s\n','}');

fprintf(fid,'%s\n',' '); % empty line
        
fprintf(fid,'\t\t%s\n','/* perform line-by-line elimination */');
fprintf(fid,'\t\t%s\n','for (iRow = 0; iRow < dim; iRow++){');
fprintf(fid,'\t\t\t%s\n','if (iRow != diagIndex){');
fprintf(fid,'\t\t\t\t%s\n','tmpFactor = inMatrixCopy[dim*diagIndex+iRow];');

fprintf(fid,'%s\n',' '); % empty line
                
fprintf(fid,'\t\t\t\t%s\n','for(iCol = 0; iCol < dim; iCol++){');
fprintf(fid,'\t\t\t\t%s\n','inMatrixCopy[dim*iCol+iRow]  -= inMatrixCopy[dim*iCol+diagIndex]*tmpFactor;');
fprintf(fid,'\t\t\t\t%s\n','outMatrix[dim*iCol+iRow] -= outMatrix[dim*iCol+diagIndex]*tmpFactor;');
fprintf(fid,'\t\t\t\t%s\n','}');
fprintf(fid,'\t\t\t%s\n','}');
fprintf(fid,'\t\t%s\n','} /* line-by-line elimination */');

fprintf(fid,'%s\n',' '); % empty line
        
fprintf(fid,'\t%s\n','}');
fprintf(fid,'\t%s\n','free(inMatrixCopy);');
fprintf(fid,'%s\n','}');

fclose(fid);

%% Generate C header file
fid = fopen(fullfile(hdrDir,hfilename),'w+');

% Function description header
fprintf(fid,'%s\n\n',hFString);

% Include guard
fprintf(fid,'%s\n%s\n\n',...
    ['#ifndef ', upper([funname,'_h'])],...
    ['#define ', upper([funname,'_h'])]);

% Function prototype
fprintf(fid,'%s\n\n',['void ',funname,'(const double *inMatrix, double *outMatrix, int dim);']);

% Include guard
fprintf(fid,'%s\n',...
    ['#endif /*', upper([funname,'_h */'])]);

fclose(fid);

CGen.logmsg('\t%s\n',' done!');

end

 %% Definition of the function description header contents
function hStruct = createHeaderStruct(fname)
[~,hStruct.funName] = fileparts(fname);
hStruct.shortDescription = ['Compute the inverse of a positive definite square matrix'];
hStruct.calls = [hStruct.funName,'(const double* inMatrix, double* outMatrix, int dim)'];
hStruct.detailedDescription = {'Given a positive definite square matrix of dimension dim in inMatrix,',...
    'the function returns its inverse outMatrix. The inverse is computed using Gauss-Jordan elimination ',...
    'without pivoting.'};
hStruct.inputs = { ['inMatrix: array of doubles storing [dim x dim] elements of the input square matrix column by column,'],...
                    'dim: dimension of the square matrix.'};
hStruct.outputs = {['outMatrix: array of doubles storing [dim x dim] elements of the output square matrix column by column.']};
hStruct.references = {'The C Book: http://publications.gbdirect.co.uk/c_book/'};
hStruct.authors = {'This is an autogenerated function!',...
    'Code generator written by:',...
    'Joern Malzahn (joern.malzahn@tu-dortmund.de)'};
hStruct.seeAlso = {'CodeGenerator'};
end