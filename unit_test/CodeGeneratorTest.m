% This is for testing the CodeGenerator functions in the robotics Toolbox
function test_suite = CodeGeneratorTest
initTestSuite;

function [tStruct] = setup
tStruct = struct;
mdl_puma560_3;
tStruct.rob = p560;
% mdl_twolink
% tStruct.rob = twolink;
tStruct.nTrials = 1000; % number of tests to perform in each subroutine

tStruct.cGen = CodeGenerator(tStruct.rob,'default','logfile','cGenUnitTestLog.txt');
tStruct.cGen.verbose = 0;

function teardown(tStruct)
clear mex
if ~isempty(strfind(path,tStruct.cGen.basepath))
    rmpath(tStruct.cGen.basepath)
    rmdir(tStruct.cGen.basepath, 's')
    delete(tStruct.cGen.logfile);
end

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

function genjacobian_test(tStruct)
% - test differential kinematics against numeric version
[J0, Jn] = tStruct.cGen.genjacobian;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
symQ = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.jacob0(q), tStruct.rob.jacob0(q));
assertElementsAlmostEqual(specRob.jacob0(q), subs(J0,symQ,q));

assertElementsAlmostEqual(specRob.jacobn(q), tStruct.rob.jacobn(q));
assertElementsAlmostEqual(specRob.jacobn(q), subs(Jn,symQ,q));


function geninertia_test(tStruct)
% - test inertial matrix against numeric version
[I] = tStruct.cGen.geninertia;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
symQ = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.inertia(q), tStruct.rob.inertia(q));
assertElementsAlmostEqual(specRob.inertia(q), subs(I,symQ,q));

function gencoriolis_test(tStruct)
% - test coriolis matrix against numeric version
[C] = tStruct.cGen.gencoriolis;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
qd = rand(1,specRob.n);
[symQ symQD] = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.coriolis(q,qd), tStruct.rob.coriolis(q,qd));
assertElementsAlmostEqual(specRob.coriolis(q,qd), subs(subs(C,symQ,q),symQD,qd));

function gengravload_test(tStruct)
% - test vector of gravitational load against numeric version
[g] = tStruct.cGen.gengravload;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
symQ = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.gravload(q), tStruct.rob.gravload(q));
assertElementsAlmostEqual(specRob.gravload(q), subs(g,symQ,q));

function genfriction_test(tStruct)
% - test friction vector against numeric version
[F] = tStruct.cGen.genfriction;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

qd = rand(1,specRob.n);
[~,symQD] = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.friction(qd), tStruct.rob.friction(qd));
assertElementsAlmostEqual(specRob.friction(qd), subs(F,symQD,qd));

function geninvdyn_test(tStruct)
% - test inverse dynamics against numeric version
tau = tStruct.cGen.geninvdyn;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
qd = rand(1,specRob.n);
qdd = rand(1,specRob.n);
[symQ,symQD,symQDD] = tStruct.rob.gencoords;

assertElementsAlmostEqual(specRob.invdyn(q,qd,qdd), tStruct.rob.rne(q,qd,qdd).');

symTau = subs(tau,symQ,q);
symTau = subs(symTau,symQD,qd);
symTau = subs(symTau,symQDD,qdd);
assertElementsAlmostEqual(specRob.invdyn(q,qd,qdd), symTau);

function genfdyn_test(tStruct)
% - test forward dynamics against numeric version
Iqdd = tStruct.cGen.genfdyn;

addpath(tStruct.cGen.basepath);
specRob = eval(tStruct.cGen.getrobfname);

q = rand(1,specRob.n);
qd = rand(1,specRob.n);
tau = rand(1,specRob.n);
[symQ,symQD] = tStruct.rob.gencoords;
symTau = tStruct.rob.genforces;

assertElementsAlmostEqual(specRob.accel(q,qd,tau), tStruct.rob.accel(q,qd,tau));

symIQdd = subs(Iqdd,symQ,q);
symIQdd = subs(symIQdd,symQD,qd);
symIQdd = subs(symIQdd,symTau,tau);
assertElementsAlmostEqual(specRob.Iqdd(q,qd,tau), symIQdd);