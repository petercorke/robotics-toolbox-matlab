%% This is for testing the SerialLink functions in the robotics Toolbox
function test_suite = TestRobotToolboxSerialLink
  initTestSuite;
  
%%    Link                       - construct a robot link object
function Test_Link
    %create a 2 link arm with Link
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
       
% Methods::
%  A             return link transform (A) matrix
    %Transforms for link 1
    expected_out = [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403    0.5000
                         0         0         0    1.0000]; 
    assertElementsAlmostEqual(L(1).A(0.5),expected_out,'absolute',1e-4);
    expected_out = [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403         0
                         0         0         0    1.0000]; 
    assertElementsAlmostEqual(L(1).A(0),expected_out,'absolute',1e-4);
    %Transforms for link 2
    expected_out = [0.8776   -0.2590    0.4034         0
                    0.4794    0.4742   -0.7385         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000]; 
    assertElementsAlmostEqual(L(2).A(0.5),expected_out,'absolute',1e-4);
    expected_out = [1.0000         0         0         0
                         0    0.5403   -0.8415         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000]; 
    assertElementsAlmostEqual(L(2).A(0),expected_out,'absolute',1e-4);
    
%  RP            return joint type: 'R' or 'P'
    expected_out = 'PR';
    assertEqual(L.RP,expected_out);
%  friction      return friction force
    L(1).B = 0.1;
    expected_out = -0.1000;
    assertElementsAlmostEqual(L(1).friction(1),expected_out,'absolute',1e-4);
    % test Coulomb friction
    L(2).Tc = [2,-1];
    expected_out = -2.000;
    assertElementsAlmostEqual(L(2).friction(1),expected_out,'absolute',1e-4);
    expected_out = 1.000;
    assertElementsAlmostEqual(L(2).friction(-2),expected_out,'absolute',1e-4);
    
%  nofriction    remove joint friction
    Ln = L(2).nofriction();
    out = Ln.friction(2);
    expected_out = 0.0000;
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    Ln = L(1).nofriction('all');
    out = Ln.friction(2);
    expected_out = 0.0000;
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);    
    
%  dyn           display link dynamic parameters
    L.dyn;
    
%  islimit       true if joint exceeds soft limit
    L(2).qlim = [-1 1];
    expected_out = 0;
    assertEqual(L(2).islimit(0),expected_out);
    L(2).qlim = [-1 1];
    expected_out = 1;
    assertEqual(L(2).islimit(3),expected_out);
    L(2).qlim = [-1 1];
    expected_out = -1;
    assertEqual(L(2).islimit(-2),expected_out);

%  isrevolute    true if joint is revolute
    assertFalse(L(1).isrevolute);
    assertTrue(L(2).isrevolute);
    
%  isprismatic   true if joint is prismatic
    assertFalse(L(2).isprismatic);
    assertTrue(L(1).isprismatic);

%  display       print the link parameters in human readable form
    L.display;
%  char          convert the link parameters to human readable string
    L.char;
%% Serial-link manipulator
%    SerialLink                 - construct a serial-link robot object
function Test_SerialLink
    % test making a robot from links 
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
    'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    % test makin a robot from DH matrix
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
    
%    *                          - compound two robots
function Test_compund
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
function Test_fkine
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
%        ikine                  - inverse kinematics (numeric)
function Test_ikine
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    T = p560.fkine(qn);
    out = p560.ikine(T);
    expected_out = [1.0511 -2.0403 0.9555 -1.1080 1.7899 -0.1130];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        ikine6s                - inverse kinematics for 6-axis arm with sph.wrist
function Test_ikine6s
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
function Test_jacob0
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
function Test_jacobn
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

%
%%     Dynamics methods
%        accel                  - forward dynamics
function Test_accel
    mdl_puma560;
    qd = 0.5 * [1 1 1 1 1 1];
    qz = [0 1 0 0 2 0];
    [Q,g]= p560.rne(qn,qz,qz);
    out = p560.accel(qz, qd,Q);
    expected_out = [-8.4837
                     3.8220
                     4.2202
                    -4.3813
                     2.2897
                    -1.5739];
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
    expected_out = [ -8.5399    3.7691    4.3488   -4.4266    2.2712   -1.5677
                     -8.0604   -5.5502    7.2075   -4.5030    1.1497   -1.5667];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        cinertia               - Cartesian manipulator inertia matrix
function Test_cinertia
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
function Test_coriolis
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
    
    
%        fdyn                   - forward dynamics
function Test_fdyn
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qd = 0.5 * [1 1 1 1 1 1];
    qdo = [0 0 0 0 0 0];
    T = 0.0001;
    
    [TI,Q,QD] = p560.fdyn(T, 0, qn, qdo);

%        gravload               - gravity loading
function Test_gravload
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
function Test_inertia
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
function Test_itorque
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qdd = 0.5 * [1 1 1 1 1 1];
    out = p560.itorque(qn,qdd);
    expected_out = [1.6763    2.1799    0.6947    0.0944    0.0861    0.0971];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%        rne                    - inverse dynamics
function Test_rne
mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.rne(qn,qz,qz);
    expected_out = [-0.9748   49.7218    5.9889   -0.0108    1.0445    0.0001];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
