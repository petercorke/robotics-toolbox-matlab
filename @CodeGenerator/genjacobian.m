%CODEGENERATOR.GENJACOBIAN Generate code for robot Jacobians
%
% J0 = cGen.genjacobian() is the symbolic expression for the Jacobian 
% matrix (6xN) expressed in the base coordinate frame.
%
% [J0, Jn] = cGen.genjacobian() as above but also returns the symbolic 
% expression for the Jacobian matrix (6xN) expressed in the end-effector 
% frame.
%
% Notes::
% - Side effects of execution depends on the cGen flags:
%   - saveresult: the symbolic expressions are saved to
%     disk in the directory specified by cGen.sympath
%   - genmfun: ready to use m-functions are generated and
%     provided via a subclass of SerialLink stored in cGen.robjpath
%   - genslblock: a Simulink block is generated and stored in a
%     robot specific block library cGen.slib in the directory
%     cGen.basepath
%   - genccode: generates C-functions and -headers in the directory 
%     specified by the ccodepath property of the CodeGenerator object.
%   - mex: generates robot specific MEX-functions as replacement for the 
%     m-functions mentioned above. Access is provided by the SerialLink 
%     subclass. The MEX files rely on the C code generated before.
%
% Author::
%  Joern Malzahn, (joern.malzahn@tu-dortmund.de)
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.genfkine.

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
%
% The code generation module emerged during the work on a project funded by
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully 
% acknowledge the financial support.

function [J0, Jn] = genjacobian(CGen)

%% Derivation of symbolic expressions
CGen.logmsg([datestr(now),'\tDeriving robot jacobians']);

q = CGen.rob.gencoords;
J0 = CGen.rob.jacob0(q);
Jn = CGen.rob.jacobn(q);

CGen.logmsg('\t%s\n',' done!');

%% Save symbolic expressions
if CGen.saveresult
    CGen.logmsg([datestr(now),'\tSaving symbolic robot jacobians']);
    
    CGen.savesym(J0,'jacob0','jacob0.mat');
    CGen.savesym(Jn,'jacobn','jacobn.mat');
    
    CGen.logmsg('\t%s\n',' done!');
end

% M-Functions
if CGen.genmfun
    CGen.genmfunjacobian;
end

% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    CGen.genslblockjacobian;
end

%% C-Code
if CGen.genccode
    CGen.genccodejacobian;
end

%% MEX
if CGen.genmex
    CGen.genmexjacobian;
end

end
