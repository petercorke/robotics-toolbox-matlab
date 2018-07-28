clc;
fclose all;
doDebug = 1;

% Create a test robot based on the first three links of the Puma 560.
    deg = pi/180;
    L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'I', [0, 0.35, 0, 0, 0, 0], ...
    'r', [0, 0, 0], ...
    'm', 0, ...
    'Jm', 200e-6, ...
    'G', -62.6111, ...
    'B', 1.48e-3, ...
    'Tc', [0.395 -0.435], ...
    'qlim', [-160 160]*deg );

    L(2) = Revolute('d', 0, 'a', 0.4318, 'alpha', 0, ...
    'I', [0.13, 0.524, 0.539, 0, 0, 0], ...
    'r', [-0.3638, 0.006, 0.2275], ...
    'm', 17.4, ...
    'Jm', 200e-6, ...
    'G', 107.815, ...
    'B', .817e-3, ...
    'Tc', [0.126 -0.071], ...
    'qlim', [-45 225]*deg );
    
    L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2,  ...
    'I', [0.066, 0.086, 0.0125, 0, 0, 0], ...
    'r', [-0.0203, -0.0141, 0.070], ...
    'm', 4.8, ...
    'Jm', 200e-6, ...
    'G', -53.7063, ...
    'B', 1.38e-3, ...
    'Tc', [0.132, -0.105], ...
    'qlim', [-225 45]*deg );

    testRob = SerialLink(L, 'name', 'UnitTestRobot');
    testRob  = testRob.nofriction('all');

    nTrials = 10; % number of tests to perform in each subroutine
    
    cGen = CodeGenerator(testRob,'default','logfile','cGenUnitTestLog.txt');
    cGen.genslblock = 0;
    
    
    %%
    % - test forward dynamics against numeric version
    IqddSym = cGen.genfdyn.';
    
    addpath(cGen.basepath);
    
    specRob = eval(cGen.getrobfname);
    [symQ, symQD] = testRob.gencoords;
    symTau = testRob.genforces;
    
    Q = rand(nTrials,specRob.n);
    QD = rand(nTrials,specRob.n);
    TAU = rand(nTrials,specRob.n);
    resRTB = zeros(specRob.n,1,nTrials);
    resSym = zeros(specRob.n,1,nTrials);
    resM = zeros(specRob.n,1,nTrials);
    resMEX = zeros(specRob.n,1,nTrials);
    
    if doDebug
    delete('numbers_ccode.txt')
    delete('numbers_matlab.txt')
    
    fid = fopen('numbers_matlab.txt','w');
    end

    profile on
    % test symbolics and generated m-code
    for iTry = 1:nTrials
        q = Q(iTry,:);
        qd = QD(iTry,:);
        tau = TAU(iTry,:);
        
        resRTB(:,:,iTry) =  testRob.accel(q,qd,tau);
        resSym(:,:,iTry) = subs(subs(subs(IqddSym,symQ,q),symQD,qd),symTau,tau);
        resM(:,:,iTry) = specRob.accel(q, qd, tau);
     
        if doDebug
            inertia = testRob.inertia(q);
            invinertia = inv(inertia);
            
            coriolis = testRob.coriolis(q, qd)*qd.';
%             tmpTau = tau  - testRob.coriolis(q, qd)*qd.' -  testRob.gravload(q) +  testRob.friction(qd);
            tmpTau = coriolis;
            gload =  testRob.gravload(q);
            fric =  testRob.friction(qd);
            for i = 1:numel(q)
                tmpTau(i) = tau(i)  - tmpTau(i) -  gload(i) + fric(i);
            end
            fprintf(fid,'coriolis: %f %f %f\n', coriolis(1), coriolis(2), coriolis(3));
            
            fprintf(fid,'\n\n');
            
            fprintf(fid,'q: %f %f %f\n', q(1),q(2),q(3));
            fprintf(fid,'qd: %f %f %f\n', qd(1),qd(2),qd(3));
            fprintf(fid,'tau: %f %f %f\n', tau(1),tau(2),tau(3));
            
            fprintf(fid,'Inertia 1: %f %f %f\n', inertia(1,1),inertia(1,2),inertia(1,3));
            fprintf(fid,'Inertia 2: %f %f %f\n', inertia(2,1),inertia(2,2),inertia(2,3));
            fprintf(fid,'Inertia 3: %f %f %f\n', inertia(3,1),inertia(3,2),inertia(3,3));
            
            fprintf(fid,'\n\n');
            
            fprintf(fid,'Inv Inertia 1: %f %f %f\n', invinertia(1,1),invinertia(1,2),invinertia(1,3));
            fprintf(fid,'Inv Inertia 2: %f %f %f\n', invinertia(2,1),invinertia(2,2),invinertia(2,3));
            fprintf(fid,'Inv Inertia 3: %f %f %f\n', invinertia(3,1),invinertia(3,2),invinertia(3,3));
            
            fprintf(fid,'\n\n');
            
            fprintf(fid,'QDD: %f %f %f\n', resRTB(1,1,iTry), resRTB(2,1,iTry), resRTB(3,1,iTry));
            
            fprintf(fid,'\n\n');
            
            fprintf(fid, 'tmpTau: %f %f %f\n', tmpTau(1), tmpTau(2), tmpTau(3));
            
            fprintf(fid,'\n\n');
            
            fprintf(fid,'\n ------------------------------------------- \n');
        end
    end
    profile off;
    
    if doDebug
        fclose(fid);
    end
    pstat = profile('info');
    statRTB = getprofilefunctionstats(pstat,['SerialLink',filesep,'accel']);
    statSym = getprofilefunctionstats(pstat,['sym',filesep,'subs']);
    statM = getprofilefunctionstats(pstat,[cGen.getrobfname,filesep,'accel']);
    profile clear;
    clear('specRob');
    rmpath(cGen.basepath)
    
    
    
%     % assertions so far?
%     verifyEqual(testCase, resRTB, resM, 'absTol', 1e-6);
%     % verifyEqual(testCase, resRTB, resSym);
    %%
    cGen.genccodefdyn;
    cGen.genmexfdyn;
    
    
    %%
    addpath(cGen.basepath);
    specRob = eval(cGen.getrobfname);
    
    profile on;
    % test generated mex code
    
%     delete('zahlen.txt')
%     delete('zahlenmatlab.txt')
    
%     fid = fopen('zahlenmatlab.txt','w');
%%
    for iTry = 1:nTrials
        q = Q(iTry,:);
        qd = QD(iTry,:);
        tau = TAU(iTry,:);
        
        resMEX(:,:,iTry) = specRob.accel(q,qd,tau);

%         disp('nothing')
%         inertia = specRob.inertia(q);
%         invinertia = inv(inertia);
        
%         fprintf(fid,'\n ------------------------------------------- \n');
%         
%         fprintf(fid,'q: %f %f %f\n', q(1),q(2),q(3));
%         fprintf(fid,'qd: %f %f %f\n', qd(1),qd(2),qd(3));
%         fprintf(fid,'tau: %f %f %f\n', tau(1),tau(2),tau(3));
%         
%         fprintf(fid,'Inertia 1: %f %f %f\n', inertia(1,1),inertia(1,2),inertia(1,3));
%         fprintf(fid,'Inertia 2: %f %f %f\n', inertia(2,1),inertia(2,2),inertia(2,3));
%         fprintf(fid,'Inertia 3: %f %f %f\n', inertia(3,1),inertia(3,2),inertia(3,3));
%         
%         fprintf(fid,'\n\n');
%         
%         fprintf(fid,'Inv Inertia 1: %f %f %f\n', invinertia(1,1),invinertia(1,2),invinertia(1,3));
%         fprintf(fid,'Inv Inertia 2: %f %f %f\n', invinertia(2,1),invinertia(2,2),invinertia(2,3));
%         fprintf(fid,'Inv Inertia 3: %f %f %f\n', invinertia(3,1),invinertia(3,2),invinertia(3,3));
%         
%         fprintf(fid,'QDD: %f %f %f\n', resMEX(1,1,iTry), resMEX(2,1,iTry), resMEX(3,1,iTry));
% 
%         fprintf(fid,'\n\n');

    end
%     fclose(fid);
    
    
    profile off;
    pstat = profile('info');
    statMEX = getprofilefunctionstats(pstat,[cGen.getrobfname,filesep,'accel.',mexext],'mex-function');
    
    z = abs(resRTB-resMEX);
    max(z(:))
%     verifyEqual(testCase, resRTB, resMEX, 'absTol', 1e-6);
    
    tRTB = statRTB.TotalTime/statRTB.NumCalls;
    tSym = statSym.TotalTime/statSym.NumCalls;
    tM = statM.TotalTime/statM.NumCalls;
    tMEX = statMEX.TotalTime/statMEX.NumCalls;
    
    fprintf('RTB function time(testCase): %f\n', tRTB)
    fprintf('Sym function time(testCase): %f  speedups: %f to RTB\n',tSym, tRTB/tSym);
    fprintf('M function time(testCase): %f  speedups: %f  to RTB,  %f  to Sym\n',tM, tRTB/tM, tSym/tM);
    fprintf('MEX function time(testCase): %f  speedups: %f  to RTB,  %f  to Sym, %f to M\n',tMEX, tRTB/tMEX, tSym/tMEX, tM/tMEX);