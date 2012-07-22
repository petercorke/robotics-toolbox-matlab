%% This is for testing the SerialLink functions in the robotics Toolbox
function test_suite = TestRobotToolboxSerialLink
  initTestSuite;
  
%% Serial-link manipulator
%    SerialLink                 - construct a serial-link robot object
function SerialLink_test
    % test making a robot from links 
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    % test makin a robot from DH matrix
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
    
function SerialLinkMDH_test
    % minimalistic test of modified DH functionality
    %  most code is common, need to exercise MDH code in Link and also for
    %  RNE

    mdl_puma560akb
    
    % qr is not set by the script, it clashes with the builtin function QR
    qz = [0 0 0 0 0 0];
   
    T = p560m.fkine(qz);
    J = p560m.jacobn(qz);
    tau = p560m.rne(qz, qz, qz);
        
%    *                          - compound two robots
function compound_test
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
    %Test two different compunding situations 
    R3 = R1*R2;
    R4 = R1*R3;


%%     Kinematic methods
%        fkine                  - forward kinematics
function fkine_test
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    out = R1.fkine([1,1]);
    expected_out = [-0.6383    0.4326    0.6368    1.2484
                    -0.0847   -0.8616    0.5004    0.3868
                     0.7651    0.2654    0.5866    2.5403
                          0         0         0    1.0000];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
    mdl_puma560
    q=[0 0 0 0 0 0; 0 0.03 -0.0365 0 0 0];
    T=p560.fkine(q);
    expected_out = [4 4 2];
    assertElementsAlmostEqual(size(T),expected_out,'absolute',1e-4);
    
%    plot                       - plot/animate robot
function SerialLink_plot_test
    L(1)=Link([1 1 1 1 1]);
    L(1).qlim = [-5 5];
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.plot([1 1]);

%    teach                      - drive a graphical  robot
function teach_test
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    L(1).qlim = [-5 5];
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    h = R1.teach;
    drawnow
    pause(0.5);
    delete(h);


%        ikine                  - inverse kinematics (numeric)
function ikine_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    T = p560.fkine(qn);
    out = p560.ikine(T);
    expected_out = [-0.0000   -0.8335    0.0940   -0.6880   -1.2140    1.1131];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

    assertTrue( all(all( abs(p560.fkine(out) - T) < 1e-4)) );

%        ikine6s                - inverse kinematics for 6-axis arm with sph.wrist
function ikine6s_test
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    T = p560.fkine(qz);
    out = p560.ikine6s(T,'ru');
    expected_out = [-1.3416 2.1416 -3.0476 2.9626 2.2600 1.6876];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
    T = p560.fkine(qn);
    out = p560.ikine6s(T,'ld');
    expected_out = [2.6486 -2.3081 3.1416 -0.1933 0.9810 -2.3371];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        jacob0                 - Jacobian in base coordinate frame
function jacob0_test
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
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    out = p560.jacob0(qn);
    expected_out = [0.1501    0.0144    0.3197         0         0         0
                    0.5963   -0.0000   -0.0000         0         0         0
                   -0.0000    0.5963    0.2910         0         0         0
                    0.0000   -0.0000   -0.0000    0.7071   -0.5950    0.7702
                   -0.0000   -1.0000   -1.0000         0   -0.5403   -0.5950
                    1.0000    0.0000    0.0000   -0.7071   -0.5950   -0.2298];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
%        jacobn                 - Jacobian in end-effector coordinate frame
function jacobn_test
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
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    out = p560.jacobn(qn);
    expected_out = [0.3893   -0.4559   -0.1506         0         0         0
                    0.4115    0.3633    0.3633         0         0         0
                   -0.2392   -0.1260    0.1793         0         0         0
                   -0.7702   -0.5950   -0.5950    0.7071         0         0
                    0.5950   -0.5403   -0.5403   -0.0000   -1.0000         0
                   -0.2298    0.5950    0.5950    0.7071    0.0000    1.0000];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%    maniplty                   - compute manipulability
function maniplty_test
    mdl_puma560;
    q = [0 pi/4 -pi 1 pi/4 0];
    assertElementsAlmostEqual(p560.maniplty(q), 0.1112, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'T'), 0.1112, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'R'), 2.5936, 'absolute',1e-4);
    assertElementsAlmostEqual(p560.maniplty(q, 'asada'), 0.2733, 'absolute',1e-4);

%
%%     Dynamics methods
%        accel                  - forward dynamics
function accel_test
    mdl_puma560;
    qd = 0.5 * [1 1 1 1 1 1];
    qz = [0 1 0 0 2 0];
    Q = p560.rne(qn,qz,qz);
    out = p560.accel(qz, qd, Q);
    expected_out = [  -9.3397 4.9666 1.6095 -5.4305 5.9885 -2.1228]';

    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

    out = p560.accel([qz, qd,Q]);
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

    out = p560.accel([qz, qd,Q]');
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

    
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
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        cinertia               - Cartesian manipulator inertia matrix
function cinertia_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.cinertia(qn);
    expected_out = [18.0741   -2.2763   -8.8446   -0.0283    0.7541   -0.4015
                    -2.2763   11.2461    1.7238   -0.0750    0.2214   -0.4733
                    -8.8446    1.7238   14.1135   -0.0300    0.7525   -0.4032
                    -0.0283   -0.0750   -0.0300    0.1377   -0.0171    0.0492
                     0.7541    0.2214    0.7525   -0.0171    0.4612   -0.2461
                    -0.4015   -0.4733   -0.4032    0.0492   -0.2461    0.3457];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);;
 

%        coriolis               - centripetal/coriolis torque
function coriolis_test
    mdl_puma560;
    qd = 0.5 * [1 1 1 1 1 1];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.coriolis(qn, qd);
    expected_out = [-0.0000   -0.9116    0.2167    0.0016   -0.0019    0.0000
                     0.3138    0.0000    0.5786   -0.0024   -0.0016    0.0000
                    -0.1805   -0.1929    0.0000   -0.0006   -0.0025    0.0000
                     0.0003    0.0009    0.0003    0.0000    0.0007   -0.0000
                     0.0001    0.0001    0.0008   -0.0006   -0.0000   -0.0000
                          0    0.0000    0.0000    0.0000    0.0000         0];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
    qd = [0.1 0.1 0.1 0.1 0.1 0.1;0.2 0.2 0.2 0.2 0.2 0.2];
    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = p560.coriolis(qn, qd);
    expected_out(:,:,1) = [-0.0000   -0.1823    0.0433    0.0003   -0.0004    0.0000
                            0.0628    0.0000    0.1157   -0.0005   -0.0003    0.0000
                           -0.0361   -0.0386    0.0000   -0.0001   -0.0005    0.0000
                            0.0001    0.0002    0.0001    0.0000    0.0001   -0.0000
                            0.0000    0.0000    0.0002   -0.0001   -0.0000   -0.0000
                                 0    0.0000    0.0000    0.0000    0.0000         0];
    expected_out(:,:,2) = [-0.0000   -0.1700   -0.0795    0.0014   -0.0003   -0.0000
                            0.0734    0.0000    0.2309   -0.0016   -0.0012    0.0000
                           -0.0021   -0.0770    0.0000   -0.0005   -0.0012    0.0000
                           -0.0001    0.0008    0.0004    0.0000    0.0004   -0.0000
                            0.0002    0.0002    0.0007   -0.0004    0.0000    0.0000
                                 0   -0.0000   -0.0000    0.0000   -0.0000         0];
    
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
    
%        gravload               - gravity loading
function gravload_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.gravload(qn);
    expected_out = [-0.0000 31.6334 6.0286 -0.0119 0.0218 0];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = p560.gravload(qn);
    expected_out = [-0.0000   31.6334    6.0286   -0.0119    0.0218         0
                     0.0000    7.7198    8.7439   -0.0238   -0.0000         0];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        inertia                - manipulator inertia matrix
function inertia_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.inertia(qn);
    expected_out = [3.6591   -0.4042    0.1013   -0.0021   -0.0015   -0.0000
                   -0.4042    4.4128    0.3504   -0.0008    0.0017    0.0000
                    0.1013    0.3504    0.9377   -0.0008    0.0008    0.0000
                   -0.0021   -0.0008   -0.0008    0.1925    0.0000    0.0000
                   -0.0015    0.0017    0.0008    0.0000    0.1713    0.0000
                   -0.0000    0.0000    0.0000    0.0000    0.0000    0.1941];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
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
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
%        itorque                - inertia torque
function itorque_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qdd = 0.5 * [1 1 1 1 1 1];
    out = p560.itorque(qn,qdd);
    expected_out = [1.6763    2.1799    0.6947    0.0944    0.0861    0.0971];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        rne                    - inverse dynamics
function rne_test
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.rne(qn,qz,qz);
    expected_out = [-0.9748   59.1306    5.9889   -0.0108    1.8872    0.0001];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

    pnf = p560.nofriction('all');

    q = rand(1,6);
    qd = rand(1,6);
    qdd = rand(1,6);

    tau1 = pnf.rne(q,qd,qdd);
    tau2 = pnf.inertia(q)*qdd' +pnf.coriolis(q,qd)*qd' + pnf.gravload(q)';
    assertElementsAlmostEqual(tau1', tau2, 'absolute', 1e-6);

%        fdyn                   - forward dynamics
function fdyn_test
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qd = 0.5 * [1 1 1 1 1 1];
    qdo = [0 0 0 0 0 0];
    T = 0.0001;

    p560 = p560.nofriction();
    
    [TI,Q,QD] = p560.fdyn(T, 0, qn, qdo);


function nofriction_test
    mdl_puma560;
    assertFalse( p560.links(3).B == 0);
    assertFalse( all(p560.links(3).Tc == 0) );

    nf = p560.nofriction();
    assertFalse( nf.links(3).B == 0);
    assertTrue( all(nf.links(3).Tc == 0) );

    mdl_puma560;
    nf = p560.nofriction('all');
    assertTrue( nf.links(3).B == 0);
    assertTrue( all(nf.links(3).Tc == 0) );
