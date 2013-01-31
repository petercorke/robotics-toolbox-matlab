%CODEGENERATOR.GENFKINE Generate code for forward kinematics
%
% T = cGen.genfkine() generates a symbolic homogeneous transform matrix (4x4) representing
% the pose of the robot end-effector in terms of the symbolic joint coordinates q1, q2, ...
%
% [T, ALLT] = cGen.genfkine() as above but also generates symbolic homogeneous transform 
% matrices (4x4xN) for the poses of the individual robot joints.
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
%
% Author::
%  Joern Malzahn
%  2012 RST, Technische Universitaet Dortmund, Germany.
%  http://www.rst.e-technik.tu-dortmund.de
%
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninvdyn, CodeGenerator.genjacobian.

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

function [t,allT] = genfkine(CGen)

%% Derivation of symbolic expressions
CGen.logmsg([datestr(now),'\tDeriving forward kinematics']);

q = CGen.rob.gencoords;
[t, allT] = CGen.rob.fkine(q);

CGen.logmsg('\t%s\n',' done!');

%% Save symbolic expressions
if CGen.saveresult
    CGen.logmsg([datestr(now),'\tSaving symbolic forward kinematics up to end-effector frame']);
    
    CGen.savesym(t,'fkine','fkine.mat')
    
    CGen.logmsg('\t%s\n',' done!');
    
    CGen.logmsg([datestr(now),'\tSaving symbolic forward kinematics for joint']);
    
    for iJoint = 1:CGen.rob.n
        CGen.logmsg(' %s ',num2str(iJoint));
        tName = ['T0_',num2str(iJoint)];
        eval([tName,' = allT(:,:,',num2str(iJoint),');']);
        CGen.savesym(eval(tName),tName,[tName,'.mat']);
    end
    
    CGen.logmsg('\t%s\n',' done!');
end

%% M-Functions
if CGen.genmfun
    CGen.genmfunfkine;
end

%% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    
    genslblockfkine(CGen);
    
end

end
