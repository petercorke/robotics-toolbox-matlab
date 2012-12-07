clear all;
clear classes;
bdclose all;
clc;

disp('Load 3-joint-Puma')
mdl_puma560_3_sym;

cGen = codeGenerator(p560,'default','logfile','myLog.txt')

cGen.genfkine;
cGen.genjacobian;
cGen.gengravload;
cGen.geninertia;

% disp('Set symbolic modeling options')
% symopt = p560.getRobotModelOpts('default');
% symopt.logFile = 'Puma3.txt';
% symopt
% p560.symopt = symopt;
% 
% 
% disp('Convert numeric Puma to symbolic Puma')
% symRob= p560.sym;
% 
% disp('Generate symbolic forward kinematics')
% [q, qd, qdd] = p560.gencoords;
% symRob.fkine(q)
% 
% %% Symbolic versions of trajectories
% % syms a b c d e f g h k
% % Q = [a b c; d e f; g h k]
% % 
% % 
% % symRob.fkine(Q)
% 
% disp('Add generated robot directory to search path.')
% addpath(symRob.getRobFName);
% 
% disp('Instantiate Puma560 object (subclass to SerialLink:)')
% subRobot = Puma560
% 
% disp('Check precompiled function:')
% H = subRobot.fkine([pi/2 0.5 0])
% 
% 
% %% Diferential Kinematics
% symRob.jacobn(q)
% 
% symRob.jacob0(q)
% 
% %% Dynamics
% 
% 
% 
