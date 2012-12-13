function [ tau ] = geninvdyn( CGen )
%% GENINVDYN Generates code from the symbolic robot specific inverse dynamics.
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
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

[q,qd,qdd] = CGen.rob.gencoords;
nJoints = CGen.rob.n;

%% Inertia matrix
CGen.logmsg([datestr(now),'\tLoading inertia matrix by row']);

I = sym(zeros(nJoints));
for kJoints = 1:nJoints
    CGen.logmsg(' %s ',num2str(kJoints));
    symname = ['inertia_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if ~exist(fname,'file')
        CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n'])
        CGen.geninertia;
    end
    tmpstruct = load(fname);
    I(kJoints,:)=tmpstruct.(symname);
    
end
CGen.logmsg('\t%s\n',' done!');

%% Matrix of centrifugal and Coriolis forces/torques matrix
CGen.logmsg([datestr(now),'\tLoading Coriolis matrix by row ']);

C = sym(zeros(nJoints));
for kJoints = 1:nJoints
    CGen.logmsg(' %s ',num2str(kJoints));
    symname = ['coriolis_row_',num2str(kJoints)];
    fname = fullfile(CGen.sympath,[symname,'.mat']);
    
    if ~exist(fname,'file')
        CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n'])
        CGen.gencoriolis;
    end
    tmpstruct = load(fname);
    C(kJoints,:)=tmpstruct.(symname);
    
end
CGen.logmsg('\t%s\n',' done!');

%% Vector of gravitational load
CGen.logmsg([datestr(now),'\tLoading vector of gravitational forces/torques']);
symname = 'gravload';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if ~exist(fname,'file')
    CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n'])
    CGen.gengravload;
end
tmpstruct = load(fname);
G = tmpstruct.(symname);

CGen.logmsg('\t%s\n',' done!');

%% Joint friction
CGen.logmsg([datestr(now),'\tLoading joint friction vector']);
symname = 'friction';
fname = fullfile(CGen.sympath,[symname,'.mat']);

if ~exist(fname,'file')
    CGen.logmsg(['\n',datestr(now),'\t Symbolics not found, generating...\n'])
    CGen.genfriction;
end
tmpstruct = load(fname);
F = tmpstruct.(symname);

CGen.logmsg('\t%s\n',' done!');

%% Full inverse dynamics
tau = I*qdd.'+C*qd.'+ G.' + F.';


%% Save symbolic expressions
if CGen.saveresult
    CGen.logmsg([datestr(now),'\tSaving symbolic inverse dynamics ']);
    
    CGen.savesym(tau,'invdyn','invdyn.mat')
    
    CGen.logmsg('\t%s\n',' done!');
end

%% M-Functions
if CGen.genmfun
    CGen.genmfuninvdyn;
end

%% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    CGen.genslblockinvdyn;
end

end
