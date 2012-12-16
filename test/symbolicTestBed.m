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
cGen.gencoriolis;
cGen.genfriction;
cGen.geninvdyn;
cGen.genfdyn;
