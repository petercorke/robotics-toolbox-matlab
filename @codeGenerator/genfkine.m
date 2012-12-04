function [t,allT] = genfkine(CGen)
%% GENFKINE Generates code from the symbolic robot specific forward kinematics expression.
%
%  Authors::
%        Jörn Malzahn
%        2012 RST, Technische Universität Dortmund, Germany
%        http://www.rst.e-technik.tu-dortmund.de
%

%% Derivation of symbolic expressions
q = CGen.rob.gencoords;
[t, allT] = CGen.rob.fkine(q);

%% Save symbolic expressions
if CGen.saveresult || CGen.genmfun || CGen.genslblock
    CGen.savesym(t,'fkine','fkine.mat')
    
    for iJoint = 1:CGen.rob.n
        tName = ['T0_',num2str(iJoint)];
        eval([tName,' = allT(:,:,',num2str(iJoint),');']);
        CGen.savesym(eval(tName),tName,[tName,'.mat']);
    end
end

%% M-Functions
if CGen.genmfun
    if ~exist(fullfile(CGen.robjpath,CGen.getrobfname),'file')
        CGen.createmconstructor;
    end
    
    CGen.genmfunfkine;
    
end

%% Embedded Matlab Function Simulink blocks
if CGen.genslblock
    
    genslblockfkine(CGen);
    
end

end
