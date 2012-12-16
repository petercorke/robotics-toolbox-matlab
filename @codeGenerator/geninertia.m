function [inertia] = geninertia(CGen)
%% GENINERTIA Generates code from the symbolic robot specific inertia matrix.
%
%  [I] = geninertia(cGen)
%  [I] = cGen.geninertia
%
%  Inputs::
%       cGen:  a codeGenerator class object
%
%       If cGen has the active flag:
%           - saveresult: the symbolic expressions are saved to
%           disk in the directory specified by cGen.sympath
%
%           - genmfun: ready to use m-functions are generated and
%           provided via a subclass of SerialLink stored in cGen.robjpath
%
%           - genslblock: a Simulink block is generated and stored in a
%           robot specific block library cGen.slib in the directory
%           cGen.basepath
%
%       The inertia matrix is stored row by row to avoid memory issues.
%       The generated code recombines these rows to output the full matrix. 
%
%  Outputs::
%       I: the symbolic robot inertia matrix (nxn)
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%
%  See also codeGenerator, geninvdyn, genfdyn

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


%% Derivation of symbolic expressions
CGen.logmsg([datestr(now),'\tDeriving robot inertia matrix']);

nJoints = CGen.rob.n;
q = CGen.rob.gencoords;
inertia = CGen.rob.inertia(q);

CGen.logmsg('\t%s\n',' done!');

%% Save symbolic expressions
if CGen.saveresult
    CGen.logmsg([datestr(now),'\tSaving rows of the inertia matrix']);
    for kJoints = 1:nJoints
        CGen.logmsg(' %i ',kJoints);           
        inertiaRow = inertia(kJoints,:);
        symName = ['inertia_row_',num2str(kJoints)];
        CGen.savesym(inertiaRow,symName,[symName,'.mat']);
    end
    CGen.logmsg('\t%s\n',' done!');   
end

% M-Functions
if CGen.genmfun
    CGen.genmfuninertia;
end

% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    CGen.genslblockinertia;
end

end