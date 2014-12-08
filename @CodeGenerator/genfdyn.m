%CODEGENERATOR.GENFDYN Generate code for forward dynamics
%
% Iqdd = cGen.genfdyn() is a symbolic vector (1xN) of joint inertial 
% reaction forces/torques.
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
% See also CodeGenerator.CodeGenerator, CodeGenerator.geninertia, CodeGenerator.genfkine.

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

function [Iqdd] = genfdyn(CGen)
[q,qd] = CGen.rob.gencoords;
tau = CGen.rob.genforces;
nJoints = CGen.rob.n;

CGen.logmsg([datestr(now),'\tLoading required symbolic expressions\n']);

%% Inertia matrix
CGen.logmsg([datestr(now),'\tLoading inertia matrix row by row']);

I = sym(zeros(nJoints));
for kJoints = 1:nJoints
    CGen.logmsg(' %s ',num2str(kJoints));
    symname = ['inertia_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if ~exist(fname,'file')
        CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n']);
        CGen.geninertia;
    end
    tmpstruct = load(fname);
    I(kJoints,:)=tmpstruct.(symname);
    
end
CGen.logmsg('\t%s\n',' done!');

%% Matrix of centrifugal and Coriolis forces/torques matrix
CGen.logmsg([datestr(now),'\t\tCoriolis matrix by row']);

C = sym(zeros(nJoints));
for kJoints = 1:nJoints
    CGen.logmsg(' %s ',num2str(kJoints));
    symname = ['coriolis_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if ~exist(fname,'file')
        CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n']);
        CGen.gencoriolis;
    end
    tmpstruct = load(fname);
    C(kJoints,:)=tmpstruct.(symname);
    
end
CGen.logmsg('\t%s\n',' done!');

%% Vector of gravitational load
CGen.logmsg([datestr(now),'\t\tvector of gravitational forces/torques']);
symname = 'gravload';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if ~exist(fname,'file')
    CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n']);
    CGen.gengravload;
end
tmpstruct = load(fname);
G = tmpstruct.(symname);

CGen.logmsg('\t%s\n',' done!');

%% Joint friction
CGen.logmsg([datestr(now),'\t\tjoint friction vector']);
symname = 'friction';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if ~exist(fname,'file')
    CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n']);
    CGen.genfriction;
end
tmpstruct = load(fname);
F = tmpstruct.(symname);

CGen.logmsg('\t%s\n',' done!');

% Full inverse dynamics
CGen.logmsg([datestr(now),'\tGenerating symbolic inertial reaction forces/torques expression\n']);
Iqdd = tau.'-C*qd.' -G.' +F.';
Iqdd = Iqdd.';

%% Save symbolic expressions
if CGen.saveresult
    CGen.logmsg([datestr(now),'\tSaving symbolic inertial reaction forces/torques expression']);
    
    CGen.savesym(Iqdd,'Iqdd','Iqdd.mat')
    
    CGen.logmsg('\t%s\n',' done!');
end

%% M-Functions
if CGen.genmfun
    CGen.genmfunfdyn;
end

%% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    CGen.genslblockfdyn;
end

%% C-Code
if CGen.genccode
    CGen.genccodefdyn;
end

%% MEX
if CGen.genmex
    CGen.genmexfdyn;
end

end
