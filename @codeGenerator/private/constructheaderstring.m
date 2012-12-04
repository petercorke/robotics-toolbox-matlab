function [hFString] = constructheaderstring(CGen,hStruct)
%% CONSTRUCTHEADERSTRING Creates common toolbox header string.
%
%   [hFString] = constructheaderstring(CGen,hStruct)
%   [hFString] = CGen.constructheaderstring(hStruct)
%
%  Description::
%    The contents of the header are coded in a struct that is the input 
%    parameter and has the following self-explaining fields:
%
%       Fieldname              Datatype
%       - funName              'string'
%       - shortDescription     'string'
%       - detailedDescription  {'line1','line2',...}
%       - inputs               {'input1: description','input2: description',...}
%       - outputs              {'output1: description','output2: description',...}
%       - examples             {'line1','line2',...}
%       - knownBugs            {'line1','line2',...}
%       - toDO                 {'line1','line2',...}
%       - references           {'line1','line2',...}
%       - authors              {'line1','line2',...}
%       - seeAlso              {'function1','function2',...}
%
%  Input::
%       hStruct:  Header content struct
%
%  Output::
%       hFString: Formatted header string
%
%  Example::
%       hStruct.funName = 'myFirstFunction';
%       hStruct.shortDescription = ['Just an example!'];
%       hStruct.calls = {'result = myFirstFunction(A,B)'};
%       constructheaderstring(hStruct)
%
%  Authors::
%        Jörn Malzahn   
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de   
%
%  See also replaceheader, sprintf.

if isfield(hStruct,'funName') && isfield(hStruct,'shortDescription')
    hFString = [];
    hFString = [hFString, sprintf('%s \n',['%% ',upper(hStruct.funName),' - ', hStruct.shortDescription, ''])];
    hFString = [hFString, sprintf('%s \n', '% =========================================================================')];
    hFString = [hFString, sprintf('%s \n', '%   ')];
else
    error('Header must include the function name and provide a short description!')
end

%% Possible Function calls
if isfield(hStruct,'calls')
    nCalls = length(hStruct.calls);
    for iCall = 1:nCalls
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.calls{iCall}])];
    end
else
    error('Header must provide function call information!')
end
hFString = [hFString, sprintf('%s \n', '%   ')];

%% Detailed description
hFString = [hFString, sprintf('%s \n', '%  Description::')];
if isfield(hStruct,'detailedDescription')
    nDescription = length(hStruct.detailedDescription);
    for iDescription = 1:nDescription
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.detailedDescription{iDescription}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end
hFString = [hFString, sprintf('%s \n', '%   ')];

%% Explanation of the Inputs
hFString = [hFString, sprintf('%s \n', '%  Input::')];
if isfield(hStruct,'inputs')
    nInputs = length(hStruct.inputs);
    for iInputs = 1:nInputs
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.inputs{iInputs}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% Explanation of the Outputs
hFString = [hFString, sprintf('%s \n', '%  Output::')];
if isfield(hStruct,'outputs')
    nOutputs = length(hStruct.outputs);
    for iOutputs = 1:nOutputs
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.outputs{iOutputs}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% Examples
hFString = [hFString, sprintf('%s \n', '%  Example::')];
if isfield(hStruct,'example')
    nExamples = length(hStruct.example);
    for iExamples = 1:nExamples
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.example{iExamples}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% Known Bugs
hFString = [hFString, sprintf('%s \n', '%  Known Bugs::')];
if isfield(hStruct,'knownBugs')
    nBugs = length(hStruct.knownBugs);
    for iBugs = 1:nBugs
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.knownBugs{iBugs}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% TODO list
hFString = [hFString, sprintf('%s \n', '%  TODO::')];
if isfield(hStruct,'toDo')
    nToDo = length(hStruct.toDo);
    for iToDo = 1:nToDo
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.toDo{iToDo}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% References
hFString = [hFString, sprintf('%s \n', '%  References::')];
if isfield(hStruct,'references')
    nRef = length(hStruct.references);
    for iRef = 1:nRef
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.references{iRef}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% Authors
hFString = [hFString, sprintf('%s \n', '%  Authors::')];
if isfield(hStruct,'authors')
    nAuthors = length(hStruct.authors);
    for iAuthors = 1:nAuthors
        hFString = [hFString, sprintf('%s \n', ['%    ',hStruct.authors{iAuthors}])];
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

%% See also
hFString = [hFString, sprintf('%s ', '%  See also')];
if isfield(hStruct,'seeAlso')
    nAlso = length(hStruct.seeAlso);
    for iAlso = 1:nAlso
        hFString = [hFString, sprintf('%s', hStruct.seeAlso{iAlso})];
        if iAlso < nAlso
            hFString = [hFString, sprintf('%s', ', ')];
        else
            hFString = [hFString, sprintf('%s\n', '.')];
        end
    end
else
    hFString = [hFString, sprintf('%s \n', '%    ---')];
end

hFString = [hFString, sprintf('%s \n', '%   ')];

hFString = [hFString, sprintf('%s \n', '   ')];