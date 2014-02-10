%CODEGENERATOR.CONSTRUCTHEADERSTRINGC Creates common toolbox header string.
%
% HFSTRING = CGen.constructheaderstringc(HSTRUCT) is the formatted header
% string.
% HSTRUCT  is the header content struct
%
% Notes::
% The contents of the header are coded in a struct that is the input 
% parameter and has the following self-explaining fields:
% Fieldname              Datatype
% - funName              'string'
% - shortDescription     'string'
% - detailedDescription  {'line1','line2',...}
% - inputs               {'input1: description','input2: description',...}
% - outputs              {'output1: description','output2: description',...}
% - examples             {'line1','line2',...}
% - knownBugs            {'line1','line2',...}
% - toDO                 {'line1','line2',...}
% - references           {'line1','line2',...}
% - authors              {'line1','line2',...}
% - seeAlso              {'function1','function2',...}
%
% Example::
% hStruct.funName = 'myFirstFunction';
% hStruct.shortDescription = ['Just an example!'];
% hStruct.calls = {'result = myFirstFunction(A,B)'};
% constructheaderstringc(hStruct)
%
% Authors::
%  Joern Malzahn   (joern.malzahn@tu-dortmund.de) 
%
% See also CodeGenerator.constructheaderstring, sprintf.

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
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

function [hFString] = constructheaderstringc(CGen,hStruct)
 
% reuse existing header construction routine,
hFString = CGen.constructheaderstring(hStruct); 

% replace comment characters for C-compliance
hFString = regexprep(hFString, '%', '//');