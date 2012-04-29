%% This is for testing the Link functions in the robotics Toolbox
function test_suite = TestRobotToolboxLink
  initTestSuite;
  
%%    Link                       - construct a robot link object
function Link_test
    %create a 2 link arm with Link
    L(1)=Link([1 1 1 1 1]);
    s = L(1).char;
    L(1).dyn;

    L(2)=Link([0 1 0 1 0]);

    s = L.char;
    L.display;
%  dyn           display link dynamic parameters
    L.dyn;
    

       
% Methods::
%  A             return link transform (A) matrix
    %Transforms for link 1
    assertElementsAlmostEqual(L(1).A(0.5), ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403    0.5000
                         0         0         0    1.0000], ...
        'absolute',1e-4);
    assertElementsAlmostEqual(L(1).A(0), ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403         0
                         0         0         0    1.0000], ...
        'absolute',1e-4);

    %Transforms for link 2
    assertElementsAlmostEqual(L(2).A(0.5), ...
    [0.8776   -0.2590    0.4034         0
                    0.4794    0.4742   -0.7385         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ... 
       'absolute',1e-4);
    assertElementsAlmostEqual(L(2).A(0), ...
                        [1.0000         0         0         0
                         0    0.5403   -0.8415         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ...
        'absolute',1e-4);
    
%  RP            return joint type: 'R' or 'P'
    assertEqual(L.RP, 'PR');

%  islimit       true if joint exceeds soft limit
    L(2).qlim = [-1 1];
    assertEqual(L(2).islimit(0), 0);
    assertEqual(L(2).islimit(3), 1);
    assertEqual(L(2).islimit(-2), -1);

%  isrevolute    true if joint is revolute
    assertFalse(L(1).isrevolute);
    assertTrue(L(2).isrevolute);
    
%  isprismatic   true if joint is prismatic
    assertFalse(L(2).isprismatic);
    assertTrue(L(1).isprismatic);

function deepcopy_test
    % deep copy test
    L = Link([1 2 3 4]);
    L2 = Link(L);
    assertEqual(L.a, 3);
    assertEqual(L2.a, 3);

    L.a = 10;
    assertEqual(L.a, 10);
    assertEqual(L2.a, 3);

function set_test
    L = Link([1 1 1 1 1]);

    L.r = [1 2 3];
    assertEqual(L.r, [1 2 3]);
    L.r = [1 2 3]';
    assertEqual(L.r, [1 2 3]);
    assertExceptionThrown(@() eval('L=Link; L.r = [ 1 2]'), 'RTB:Link:badarg');

    assertExceptionThrown(@() eval('L=Link; L.I = 1'), 'RTB:Link:badarg');
    %assertExceptionThrown(@()L.I = rand(3,3), 'RTB:Link:badarg');
    L.I = [1, 2, 3];
    assertEqual(L.I, diag([1,2,3]));
    L.I = [1, 2, 3, 4, 5, 6];
    assertEqual(L.I, [1 4 6; 4 2 5; 6 5 3]);
    A = rand(3,3);
    L.I = A'*A;     % make a symm matrix
    assertEqual(L.I, A'*A);

    L.Tc = 1;
    assertEqual(L.Tc, [1 -1]);
    L.Tc = [1 -2];
    assertEqual(L.Tc, [1 -2]);
    assertExceptionThrown(@() eval('L=Link; L.Tc = [-2 1]'), 'RTB:Link:badarg');
    
function viscous_friction_test
    L = Link();
    B = 1; G = 2; Tc = 0;

    % viscous model
    L.G = G;
    L.B = B;
    L.Tc = Tc;

    assertEqual(L.friction(1), -B*G^2);
    assertEqual(L.friction(-1), B*G^2);

    L.G = -G
    assertEqual(L.friction(1), -B*G^2);
    assertEqual(L.friction(-1), B*G^2);

function coulomb_friction_test
    L = Link();
    B = 0; G = 2; Tc = [2 -3];

    % coulomb model
    L.G = G
    L.B = 0;
    L.Tc = Tc;

    assertEqual(L.friction(1), -Tc(1)*G);
    assertEqual(L.friction(-1), -Tc(2)*G);

    % check for negative gear ratio
    L.G = -G

    % coulomb model
    L.B = 0;
    L.Tc = Tc;

    assertEqual(L.friction(1), -Tc(1)*G);
    assertEqual(L.friction(-1), -Tc(2)*G);


%  nofriction    remove joint friction
function nofriction_test
    L = Link();
    B = 1; G = 2; Tc = [2 -3];
    L.B = B;
    L.G = G
    L.Tc = Tc;

    assertFalse( L.B == 0);
    assertFalse( all(L.Tc == 0) );

    Lnf = L.nofriction('all');
    assertTrue( Lnf.B == 0);
    assertTrue( all(Lnf.Tc == 0) );
    % check original object unchanged
    assertFalse( L.B == 0);
    assertFalse( all(L.Tc == 0) );

    Lnf = L.nofriction('viscous');
    assertTrue( Lnf.B == 0);
    assertFalse( all(Lnf.Tc == 0) );

    Lnf = L.nofriction('coulomb');
    assertFalse( Lnf.B == 0);
    assertTrue( all(Lnf.Tc == 0) );
