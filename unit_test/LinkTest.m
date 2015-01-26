%% This is for testing the Link functions in the robotics Toolbox
function tests = LinkTest
  tests = functiontests(localfunctions);
end
  
%%    Link                       - construct a robot link object
function Link_test(testCase)
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
    verifyEqual(testCase, L(1).A(0.5), ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403    0.5000
                         0         0         0    1.0000], ...
        'absTol',1e-4);
    verifyEqual(testCase, L(1).A(0), ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403         0
                         0         0         0    1.0000], ...
        'absTol',1e-4);

    %Transforms for link 2
    verifyEqual(testCase, L(2).A(0.5), ...
    [0.8776   -0.2590    0.4034         0
                    0.4794    0.4742   -0.7385         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ... 
       'absTol',1e-4);
    verifyEqual(testCase, L(2).A(0), ...
                        [1.0000         0         0         0
                         0    0.5403   -0.8415         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ...
        'absTol',1e-4);
    
%  RP            return joint type: 'R' or 'P'
    verifyEqual(testCase, L.RP, 'PR');

%  islimit       true if joint exceeds soft limit
    L(2).qlim = [-1 1];
    verifyEqual(testCase, L(2).islimit(0), 0);
    verifyEqual(testCase, L(2).islimit(3), 1);
    verifyEqual(testCase, L(2).islimit(-2), -1);

%  isrevolute    true if joint is revolute
    verifyFalse(testCase, L(1).isrevolute);
    verifyTrue(testCase, L(2).isrevolute);
    
%  isprismatic   true if joint is prismatic
    verifyFalse(testCase, L(2).isprismatic);
    verifyTrue(testCase, L(1).isprismatic);
end

function deepcopy_test(testCase)
    % deep copy test
    L = Link([1 2 3 4]);
    L2 = Link(L);
    verifyEqual(testCase, L.a, 3);
    verifyEqual(testCase, L2.a, 3);

    L.a = 10;
    verifyEqual(testCase, L.a, 10);
    verifyEqual(testCase, L2.a, 3);
end

function set_test(testCase)
    L = Link([1 1 1 1 1]);

    L.r = [1 2 3];
    verifyEqual(testCase, L.r, [1 2 3]);
    L.r = [1 2 3]';
    verifyEqual(testCase, L.r, [1 2 3]);
    verifyError(testCase, @() eval('L=Link; L.r = [ 1 2]'), 'RTB:Link:badarg');

    verifyError(testCase, @() eval('L=Link; L.I = 1'), 'RTB:Link:badarg');
    %verifyError(testCase, @()L.I = rand(3,3), 'RTB:Link:badarg');
    L.I = [1, 2, 3];
    verifyEqual(testCase, L.I, diag([1,2,3]));
    L.I = [1, 2, 3, 4, 5, 6];
    verifyEqual(testCase, L.I, [1 4 6; 4 2 5; 6 5 3]);
    A = rand(3,3);
    L.I = A'*A;     % make a symm matrix
    verifyEqual(testCase, L.I, A'*A);

    L.Tc = 1;
    verifyEqual(testCase, L.Tc, [1 -1]);
    L.Tc = [1 -2];
    verifyEqual(testCase, L.Tc, [1 -2]);
    verifyError(testCase, @() eval('L=Link; L.Tc = [-2 1]'), 'RTB:Link:badarg');
end
    
function viscous_friction_test(testCase)
    L = Link();
    B = 1; G = 2; Tc = 0;

    % viscous model
    L.G = G;
    L.B = B;
    L.Tc = Tc;

    verifyEqual(testCase, L.friction(1), -B*G^2);
    verifyEqual(testCase, L.friction(-1), B*G^2);

    L.G = -G
    verifyEqual(testCase, L.friction(1), -B*G^2);
    verifyEqual(testCase, L.friction(-1), B*G^2);
end

function coulomb_friction_test(testCase)
    L = Link();
    B = 0; G = 2; Tc = [2 -3];

    % coulomb model
    L.G = G
    L.B = 0;
    L.Tc = Tc;

    verifyEqual(testCase, L.friction(1), -Tc(1)*G);
    verifyEqual(testCase, L.friction(-1), -Tc(2)*G);

    % check for negative gear ratio
    L.G = -G

    % coulomb model
    L.B = 0;
    L.Tc = Tc;

    verifyEqual(testCase, L.friction(1), -Tc(1)*G);
    verifyEqual(testCase, L.friction(-1), -Tc(2)*G);
end


%  nofriction    remove joint friction
function nofriction_test(testCase)
    L = Link();
    B = 1; G = 2; Tc = [2 -3];
    L.B = B;
    L.G = G
    L.Tc = Tc;

    verifyFalse(testCase,  L.B == 0);
    verifyFalse(testCase,  all(L.Tc == 0) );

    Lnf = L.nofriction('all');
    verifyTrue(testCase,  Lnf.B == 0);
    verifyTrue(testCase,  all(Lnf.Tc == 0) );
    % check original object unchanged
    verifyFalse(testCase,  L.B == 0);
    verifyFalse(testCase,  all(L.Tc == 0) );

    Lnf = L.nofriction('viscous');
    verifyTrue(testCase,  Lnf.B == 0);
    verifyFalse(testCase,  all(Lnf.Tc == 0) );

    Lnf = L.nofriction('coulomb');
    verifyFalse(testCase,  Lnf.B == 0);
    verifyTrue(testCase,  all(Lnf.Tc == 0) );
end
