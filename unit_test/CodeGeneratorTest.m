% This is for testing the CodeGenerator functions in the robotics Toolbox
function test_suite = CodeGeneratorTest
initTestSuite;

function [tStruct] = setup
tStruct = struct;
mdl_puma560_3;
tStruct.rob = p560;
% mdl_twolink
% tStruct.rob = twolink;
% tStruct.nTrials = 1000; % number of tests to perform in each subroutine
tStruct.nTrials = 1; % number of tests to perform in each subroutine

tStruct.cGen = CodeGenerator(tStruct.rob,'default','logfile','cGenUnitTestLog.txt');
tStruct.cGen.verbose = 0;

function teardown(tStruct)
clear mex
if ~isempty(strfind(path,tStruct.cGen.basepath))
    rmpath(tStruct.cGen.basepath)
    rmdir(tStruct.cGen.basepath, 's')
    delete(tStruct.cGen.logfile);
end

%%
function genfkine_test(tStruct)
% - test generated forward kinematics code
T = tStruct.cGen.genfkine; % generate symbolic expressions and m-files
addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
symQ = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
resRTB = rand(4,4,tStruct.nTrials);
resSym = rand(4,4,tStruct.nTrials);
resM = rand(4,4,tStruct.nTrials);
resMEX = rand(4,4,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.fkine(q);
    resSym(:,:,iTry) = subs(T,symQ,q);
    resM(:,:,iTry) = specRob.fkine(q);
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'fkine']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'fkine']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);

tStruct.cGen.genccodefkine;
tStruct.cGen.genmexfkine;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    resMEX(:,:,iTry) = specRob.fkine(q);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'fkine.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);

tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);

%%
function genjacobian_test(tStruct)
% - test generated differential kinematics code
[J0, Jn] = tStruct.cGen.genjacobian;
addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
symQ = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
resRTB0 = rand(6,specRob.n,tStruct.nTrials);
resSym0 = rand(6,specRob.n,tStruct.nTrials);
resM0 = rand(6,specRob.n,tStruct.nTrials);
resMEX0 = rand(6,specRob.n,tStruct.nTrials);
resRTBn = rand(6,specRob.n,tStruct.nTrials);
resSymn = rand(6,specRob.n,tStruct.nTrials);
resMn = rand(6,specRob.n,tStruct.nTrials);
resMEXn = rand(6,specRob.n,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    
    resRTB0(:,:,iTry) =  tStruct.rob.jacob0(q);
    resSym0(:,:,iTry) = subs(J0,symQ,q);
    resM0(:,:,iTry) = specRob.jacob0(q);
    
    resRTBn(:,:,iTry) =  tStruct.rob.jacobn(q);
    resSymn(:,:,iTry) = subs(Jn,symQ,q);
    resMn(:,:,iTry) = specRob.jacobn(q);
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'jacob0']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'jacob0']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB0, resSym0);
assertElementsAlmostEqual(resRTB0, resM0);
assertElementsAlmostEqual(resRTBn, resSymn);
assertElementsAlmostEqual(resRTBn, resMn);

tStruct.cGen.genccodejacobian;
tStruct.cGen.genmexjacobian;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    resMEX0(:,:,iTry) = specRob.jacob0(q);
    resMEXn(:,:,iTry) = specRob.jacobn(q);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'jacob0.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB0, resMEX0);
assertElementsAlmostEqual(resRTBn, resMEXn);

tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);

%%
function geninertia_test(tStruct)
% - test inertial matrix against numeric version
[I] = tStruct.cGen.geninertia;

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
symQ = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,specRob.n,tStruct.nTrials);
resSym = rand(specRob.n,specRob.n,tStruct.nTrials);
resM = rand(specRob.n,specRob.n,tStruct.nTrials);
resMEX = rand(specRob.n,specRob.n,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.inertia(q);
    resSym(:,:,iTry) = subs(I,symQ,q);
    resM(:,:,iTry) = specRob.inertia(q);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'inertia']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'inertia']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);


tStruct.cGen.genccodeinertia;
tStruct.cGen.genmexinertia;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    resMEX(:,:,iTry) = specRob.inertia(q);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'inertia.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);


tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);


function gencoriolis_test(tStruct)
% - test coriolis matrix against numeric version
[C] = tStruct.cGen.gencoriolis;

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
[symQ symQD] = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
QD = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,specRob.n,tStruct.nTrials);
resSym = rand(specRob.n,specRob.n,tStruct.nTrials);
resM = rand(specRob.n,specRob.n,tStruct.nTrials);
resMEX = rand(specRob.n,specRob.n,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.coriolis(q,qd);
    resSym(:,:,iTry) = subs(subs(C,symQ,q),symQD,qd);
    resM(:,:,iTry) = specRob.coriolis(q,qd);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'coriolis']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'coriolis']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);


tStruct.cGen.genccodecoriolis;
tStruct.cGen.genmexcoriolis;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    resMEX(:,:,iTry) = specRob.coriolis(q,qd);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'coriolis.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);


tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);

%%
function gengravload_test(tStruct)
% - test vector of gravitational load against numeric version
[g] = tStruct.cGen.gengravload;

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
symQ = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,1,tStruct.nTrials);
resSym = rand(specRob.n,1,tStruct.nTrials);
resM = rand(specRob.n,1,tStruct.nTrials);
resMEX = rand(specRob.n,1,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.gravload(q);
    resSym(:,:,iTry) = subs(g,symQ,q);
    resM(:,:,iTry) = specRob.gravload(q);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'gravload']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'gravload']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);


tStruct.cGen.genccodegravload;
tStruct.cGen.genmexgravload;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    resMEX(:,:,iTry) = specRob.gravload(q);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'gravload.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);


tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);

function genfriction_test(tStruct)
% - test friction vector against numeric version
[F] = tStruct.cGen.genfriction;

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
[~, symQD] = tStruct.rob.gencoords;

QD = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,1,tStruct.nTrials);
resSym = rand(specRob.n,1,tStruct.nTrials);
resM = rand(specRob.n,1,tStruct.nTrials);
resMEX = rand(specRob.n,1,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    qd = QD(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.friction(qd);
    resSym(:,:,iTry) = subs(F,symQD,qd);
    resM(:,:,iTry) = specRob.friction(qd);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'friction']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'friction']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);


tStruct.cGen.genccodefriction;
tStruct.cGen.genmexfriction;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    qd = QD(iTry,:);
    resMEX(:,:,iTry) = specRob.friction(qd);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'friction.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);


tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);

function geninvdyn_test(tStruct)
% - test inverse dynamics against numeric version
tau = tStruct.cGen.geninvdyn;

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
[symQ, symQD, symQDD] = tStruct.rob.gencoords;

Q = rand(tStruct.nTrials,specRob.n);
QD = rand(tStruct.nTrials,specRob.n);
QDD = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,1,tStruct.nTrials);
resSym = rand(specRob.n,1,tStruct.nTrials);
resM = rand(specRob.n,1,tStruct.nTrials);
resMEX = rand(specRob.n,1,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    qdd = QDD(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.rne(q,qd,qdd);
    resSym(:,:,iTry) = subs(subs(subs(tau,symQ,q),symQD,qd),symQDD,qdd);
    resM(:,:,iTry) = specRob.invdyn(q, qd, qdd);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'rne']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'invdyn']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resSym);
assertElementsAlmostEqual(resRTB, resM);


tStruct.cGen.genccodeinvdyn;
tStruct.cGen.genmexinvdyn;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    qdd = QDD(iTry,:);
    
    resMEX(:,:,iTry) = specRob.invdyn(q,qd,qdd);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'invdyn.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);

tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);


function genfdyn_test(tStruct)
% - test forward dynamics against numeric version
Iqdd = tStruct.cGen.genfdyn.';

addpath(tStruct.cGen.basepath);

specRob = eval(tStruct.cGen.getrobfname);
[symQ, symQD] = tStruct.rob.gencoords;
symTau = tStruct.rob.genforces;

Q = rand(tStruct.nTrials,specRob.n);
QD = rand(tStruct.nTrials,specRob.n);
TAU = rand(tStruct.nTrials,specRob.n);
resRTB = rand(specRob.n,1,tStruct.nTrials);
resSym = rand(specRob.n,1,tStruct.nTrials);
resM = rand(specRob.n,1,tStruct.nTrials);
resMEX = rand(specRob.n,1,tStruct.nTrials);

profile on
% test symbolics and generated m-code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    tau = TAU(iTry,:);
    
    resRTB(:,:,iTry) =  tStruct.rob.accel(q,qd,tau);
    resSym(:,:,iTry) = subs(subs(subs(Iqdd,symQ,q),symQD,qd),symTau,tau);
    resM(:,:,iTry) = specRob.accel(q, qd, tau);
    
end
profile off;
pstat = profile('info');
statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'accel']);
statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
statM = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'accel']);
profile clear;
clear('specRob');
rmpath(tStruct.cGen.basepath)

% assertions so far?
assertElementsAlmostEqual(resRTB, resM);
% assertElementsAlmostEqual(resRTB, resSym);

tStruct.cGen.genccodefdyn;
tStruct.cGen.genmexfdyn;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

profile on;
% test generated mex code
for iTry = 1:tStruct.nTrials
    q = Q(iTry,:);
    qd = QD(iTry,:);
    tau = TAU(iTry,:);
    
    resMEX(:,:,iTry) = specRob.fdyn(q,qd,tau);
end
profile off;
pstat = profile('info');
statMEX = getprofilefunctionstats(pstat,[tStruct.cGen.getrobfname,filesep,'fdyn.',mexext],'mex-function');

assertElementsAlmostEqual(resRTB, resMEX);

tRTB = statRTB.TotalTime/statRTB.NumCalls;
tSym = statSym.TotalTime/statSym.NumCalls;
tM = statM.TotalTime/statM.NumCalls;
tMEX = statMEX.TotalTime/statMEX.NumCalls;

fprintf('RTB function time: %f\n', tRTB)
fprintf('Sym function time: %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
fprintf('M function time: %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
fprintf('MEX function time: %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);
