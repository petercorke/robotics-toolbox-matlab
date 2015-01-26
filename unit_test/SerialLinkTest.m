%% This is for testing the SerialLink functions in the robotics Toolbox
function tests = SerialLinkTest
  tests = functiontests(localfunctions);
end

%% Serial-link manipulator
%    SerialLink                 - construct a serial-link robot object
function SerialLink_test(testCase)
    % test making a robot from links 
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    % test makin a robot from DH matrix
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
end
    
function SerialLinkMDH_test(testCase)
    % minimalistic test of modified DH functionality
    %  most code is common, need to exercise MDH code in Link and also for
    %  RNE

    mdl_puma560akb

    % qr is not set by the script, it clashes with the builtin function QR(testCase)
    qz = [0 0 0 0 0 0];
   
    T = p560m.fkine(qz);
    J = p560m.jacobn(qz);
    tau = p560m.rne(qz, qz, qz);
end
        
%    *                          - compound two robots
function compound_test(testCase)
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
    %Test two different compunding situations 
    R3 = R1*R2;
    R4 = R1*R3;
end


%%     Kinematic methods
%        fkine                  - forward kinematics
function fkine_test(testCase)
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    out = R1.fkine([1,1]);
    expected_out = [-0.6383    0.4326    0.6368    1.2484
                    -0.0847   -0.8616    0.5004    0.3868
                     0.7651    0.2654    0.5866    2.5403
                          0         0         0    1.0000];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    
    mdl_puma560
    q=[0 0 0 0 0 0; 0 0.03 -0.0365 0 0 0];
    T=p560.fkine(q);
    expected_out = [4 4 2];
    verifyEqual(testCase, size(T),expected_out,'absTol',1e-4);
end
    
%    plot                       - plot/animate robot
function SerialLink_plot_test(testCase)
    L(1)=Link([1 1 1 1 0]);
    L(1).qlim = [-5 5];
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.plot([1 1]);
end

%    teach                      - drive a graphical  robot
function teach_test(testCase)
    L(1)=Link([1 1 1 1 0]);
    L(2)=Link([0 1 0 1 0]);
    L(1).qlim = [-5 5];
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.teach;
    pause(0.5);
end

%        ikine                  - inverse kinematics (numeric)
function ikine_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 0 pi/4 0];
    T = p560.fkine(qn);
    out = p560.ikine(T, [0 0 3 0 0 0]);
    % example from RVC p 149
    expected_out = [-0.0000    0.7854    3.1416    0.0000    0.7854   -0.0000];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

    verifyTrue(testCase,  all(all( abs(p560.fkine(out) - T) < 1e-4)) );
end

%        ikine6s                - inverse kinematics for 6-axis arm with sph.wrist
function ikine6s_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    T = p560.fkine(qn);
    out = p560.ikine6s(T,'ru');
    expected_out = [2.6486   -2.3081    3.1416    2.9483   -0.9810    0.8044];
    verifyEqual(testCase, out,expected_out,'absTol',1e-3);
    
    T = p560.fkine(qn);
    out = p560.ikine6s(T,'ld');
    expected_out = [-0.0000   -0.8335    0.0940   -0.6880   -1.2140    1.1131];
    verifyEqual(testCase, out,expected_out,'absTol',1e-3);
end

%        jacob0                 - Jacobian in base coordinate frame
function jacob0_test(testCase)
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.jacob0(qz);
    expected_out = [0.1501   -0.6137   -0.2504         0         0         0
                   -0.1191   -0.0000   -0.0000         0         0         0
                   -0.0000   -0.1191   -0.3524         0         0         0
                         0   -0.0000   -0.0000   -0.8415   -0.0000   -0.1411
                    0.0000   -1.0000   -1.0000   -0.0000   -1.0000   -0.0000
                    1.0000    0.0000    0.0000    0.5403    0.0000   -0.9900];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    out = p560.jacob0(qn);
    expected_out = [0.1501    0.0144    0.3197         0         0         0
                    0.5963   -0.0000   -0.0000         0         0         0
                   -0.0000    0.5963    0.2910         0         0         0
                    0.0000   -0.0000   -0.0000    0.7071   -0.5950    0.7702
                   -0.0000   -1.0000   -1.0000         0   -0.5403   -0.5950
                    1.0000    0.0000    0.0000   -0.7071   -0.5950   -0.2298];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end
    
%        jacobn                 - Jacobian in end-effector coordinate frame
function jacobn_test(testCase)
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.jacobn(qz);
    expected_out = [-0.1485    0.5908    0.1982         0         0         0
                    -0.1191    0.0000    0.0000         0         0         0
                    -0.0212    0.2045    0.3842         0         0         0
                     0.1411         0         0    0.9093         0         0
                    -0.0000   -1.0000   -1.0000   -0.0000   -1.0000         0
                    -0.9900    0.0000    0.0000   -0.4161    0.0000    1.0000];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    out = p560.jacobn(qn);
    expected_out = [0.3893   -0.4559   -0.1506         0         0         0
                    0.4115    0.3633    0.3633         0         0         0
                   -0.2392   -0.1260    0.1793         0         0         0
                   -0.7702   -0.5950   -0.5950    0.7071         0         0
                    0.5950   -0.5403   -0.5403   -0.0000   -1.0000         0
                   -0.2298    0.5950    0.5950    0.7071    0.0000    1.0000];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end

%    maniplty                   - compute manipulability
function maniplty_test(testCase)
    mdl_puma560;
    q = [0 pi/4 -pi 1 pi/4 0];
    verifyEqual(testCase, p560.maniplty(q), 0.1112, 'absTol',1e-4);
    verifyEqual(testCase, p560.maniplty(q, 'T'), 0.1112, 'absTol',1e-4);
    verifyEqual(testCase, p560.maniplty(q, 'R'), 2.5936, 'absTol',1e-4);
    verifyEqual(testCase, p560.maniplty(q, 'asada'), 0.2733, 'absTol',1e-4);
end

%
%%     Dynamics methods
%        accel                  - forward dynamics
function accel_test(testCase)
    mdl_puma560;
    qd = 0.5 * [1 1 1 1 1 1];
    qz = [0 1 0 0 2 0];
    Q = p560.rne(qn,qz,qz);
    out = p560.accel(qz, qd, Q);
    expected_out = [  -9.3397 4.9666 1.6095 -5.4305 5.9885 -2.1228]';

    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

    out = p560.accel([qz, qd,Q]);
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

    out = p560.accel([qz, qd,Q]');
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

    
    qd = [0.1 0.1 0.1 0.1 0.1 0.1;0.2 0.2 0.2 0.2 0.2 0.2];
    qz = [0 1 0 0 2 0;0 0.5 0 0 1 0];
    qd1 = [0.1 0.1 0.1 0.1 0.1 0.1];
    qd2 = [0.2 0.2 0.2 0.2 0.2 0.2];
    qz1 = [0 1 0 0 2 0];
    qz2 = [0 0.5 0 0 1 0];
    qn1 = [0 pi/4 -pi 1 pi/4 0];
    qn2 = [0 pi/2 -pi 1 pi/2 0];

    Q1 = p560.rne(qn1,qz1,qz1);
    Q2 = p560.rne(qn2,qz2,qz2);
    
    q3 = [Q1;Q2];
    
    out = p560.accel(qz, qd,q3);
    expected_out = [
       -8.2760    5.8119    3.1487   -4.6392    6.9558   -1.6774
          -8.3467   -4.8514    6.0575   -4.9232    3.1244   -1.7861 ];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end

%        cinertia               - Cartesian manipulator inertia matrix
function cinertia_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.cinertia(qn);
    expected_out = [18.0741   -2.2763   -8.8446   -0.0283    0.7541   -0.4015
                    -2.2763   11.2461    1.7238   -0.0750    0.2214   -0.4733
                    -8.8446    1.7238   14.1135   -0.0300    0.7525   -0.4032
                    -0.0283   -0.0750   -0.0300    0.1377   -0.0171    0.0492
                     0.7541    0.2214    0.7525   -0.0171    0.4612   -0.2461
                    -0.4015   -0.4733   -0.4032    0.0492   -0.2461    0.3457];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);;
end
 

%        coriolis               - centripetal/coriolis torque
function coriolis_test(testCase)
    mdl_puma560;
    qd = 0.5 * [1 1 1 1 1 1];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.coriolis(qn, qd);
    expected_out = [
   -0.1336   -0.6458    0.0845    0.0005   -0.0008    0.0000
    0.3136    0.1922    0.3851   -0.0018   -0.0007    0.0000
   -0.1803   -0.1934   -0.0005   -0.0009   -0.0014    0.0000
    0.0007    0.0008    0.0003    0.0001    0.0003   -0.0000
   -0.0004    0.0005    0.0005   -0.0003   -0.0000   -0.0000
    0.0000    0.0000    0.0000   -0.0000    0.0000         0
                          ];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    
    qd = [0.1 0.1 0.1 0.1 0.1 0.1;0.2 0.2 0.2 0.2 0.2 0.2];
    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = p560.coriolis(qn, qd);
    expected_out(:,:,1) = [
       -0.0267   -0.1292    0.0169    0.0001   -0.0002    0.0000
        0.0627    0.0384    0.0770   -0.0004   -0.0001    0.0000
       -0.0361   -0.0387   -0.0001   -0.0002   -0.0003    0.0000
        0.0001    0.0002    0.0001    0.0000    0.0001   -0.0000
       -0.0001    0.0001    0.0001   -0.0001   -0.0000   -0.0000
        0.0000    0.0000    0.0000   -0.0000    0.0000         0
                                 ];
    expected_out(:,:,2) = [
       -0.0715   -0.1242   -0.0534    0.0008   -0.0001   -0.0000
        0.0731    0.0765    0.1535   -0.0009   -0.0007    0.0000
       -0.0023   -0.0772   -0.0003   -0.0004   -0.0007    0.0000
        0.0004    0.0006    0.0003    0.0000    0.0002   -0.0000
        0.0002    0.0004    0.0004   -0.0002    0.0000    0.0000
       -0.0000    0.0000    0.0000   -0.0000   -0.0000         0
                                 ];
    
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end
    
    
%        gravload               - gravity loading
function gravload_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.gravload(qn);
    expected_out = [-0.0000 31.6334 6.0286 -0.0119 0.0218 0];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = p560.gravload(qn);
    expected_out = [-0.0000   31.6334    6.0286   -0.0119    0.0218         0
                     0.0000    7.7198    8.7439   -0.0238   -0.0000         0];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end

%        inertia                - manipulator inertia matrix
function inertia_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.inertia(qn);
    expected_out = [3.6591   -0.4042    0.1013   -0.0021   -0.0015   -0.0000
                   -0.4042    4.4128    0.3504   -0.0008    0.0017    0.0000
                    0.1013    0.3504    0.9377   -0.0008    0.0008    0.0000
                   -0.0021   -0.0008   -0.0008    0.1925    0.0000    0.0000
                   -0.0015    0.0017    0.0008    0.0000    0.1713    0.0000
                   -0.0000    0.0000    0.0000    0.0000    0.0000    0.1941];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    
    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = p560.inertia(qn);
    expected_out(:,:,1) = [3.6591   -0.4042    0.1013   -0.0021   -0.0015   -0.0000
                          -0.4042    4.4128    0.3504   -0.0008    0.0017    0.0000
                           0.1013    0.3504    0.9377   -0.0008    0.0008    0.0000
                          -0.0021   -0.0008   -0.0008    0.1925    0.0000    0.0000
                          -0.0015    0.0017    0.0008    0.0000    0.1713    0.0000
                          -0.0000    0.0000    0.0000    0.0000    0.0000    0.1941];
    expected_out(:,:,2) = [2.6621   -0.6880    0.0035   -0.0007   -0.0010    0.0000
                          -0.6880    4.4114    0.3487   -0.0010    0.0015    0.0000
                           0.0035    0.3487    0.9359   -0.0010    0.0003    0.0000
                          -0.0007   -0.0010   -0.0010    0.1926    0.0000    0.0000
                          -0.0010    0.0015    0.0003    0.0000    0.1713    0.0000
                           0.0000    0.0000    0.0000    0.0000    0.0000    0.1941];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end
    
%        itorque                - inertia torque
function itorque_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qdd = 0.5 * [1 1 1 1 1 1];
    out = p560.itorque(qn,qdd);
    expected_out = [1.6763    2.1799    0.6947    0.0944    0.0861    0.0971];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

%        rne                    - inverse dynamics
end
function rne_test(testCase)
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.rne(qn,qz,qz);
    expected_out = [-0.9748   59.1306    5.9889   -0.0108    1.8872    0.0001];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);

    pnf = p560.nofriction('all');

    q = rand(1,6);
    qd = rand(1,6);
    qdd = rand(1,6);

    tau1 = pnf.rne(q,qd,qdd);
    tau2 = pnf.inertia(q)*qdd' +pnf.coriolis(q,qd)*qd' + pnf.gravload(q)';
    verifyEqual(testCase, tau1', tau2, 'absTol', 1e-6);
end

%        fdyn                   - forward dynamics
function fdyn_test(testCase)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qd = 0.5 * [1 1 1 1 1 1];
    qdo = [0 0 0 0 0 0];
    T = 0.0001;

    p560 = p560.nofriction();
    
    [TI,Q,QD] = p560.fdyn(T, 0, qn, qdo);
end

function nofriction_test(testCase)
    mdl_puma560;
    verifyFalse(testCase,  p560.links(3).B == 0);
    verifyFalse(testCase,  all(p560.links(3).Tc == 0) );

    nf = p560.nofriction();
    verifyFalse(testCase,  nf.links(3).B == 0);
    verifyTrue(testCase,  all(nf.links(3).Tc == 0) );

    mdl_puma560;
    nf = p560.nofriction('all');
    verifyTrue(testCase,  nf.links(3).B == 0);
    verifyTrue(testCase,  all(nf.links(3).Tc == 0) );
end
