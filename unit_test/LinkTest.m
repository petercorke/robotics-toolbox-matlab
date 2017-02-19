%% This is for testing the Link functions in the robotics Toolbox
function tests = LinkTest
  tests = functiontests(localfunctions);
end
  
%%    Link                       - construct a robot link object
function Link_test(tc)
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
    verifyEqual(tc, L(1).A(0.5).T, ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403    0.5000
                         0         0         0    1.0000], ...
        'absTol',1e-4);
    verifyEqual(tc, L(1).A(0).T, ...
                    [0.5403   -0.4546    0.7081    0.5403
                    0.8415    0.2919   -0.4546    0.8415
                         0    0.8415    0.5403         0
                         0         0         0    1.0000], ...
        'absTol',1e-4);

    %Transforms for link 2
    verifyEqual(tc, L(2).A(0.5).T, ...
    [0.8776   -0.2590    0.4034         0
                    0.4794    0.4742   -0.7385         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ... 
       'absTol',1e-4);
    verifyEqual(tc, L(2).A(0).T, ...
                        [1.0000         0         0         0
                         0    0.5403   -0.8415         0
                         0    0.8415    0.5403    1.0000
                         0         0         0    1.0000], ...
        'absTol',1e-4);
    
%  RP            return joint type: 'R' or 'P'
    verifyEqual(tc, L.type, 'PR');

%  islimit       true if joint exceeds soft limit
    L(2).qlim = [-1 1];
    verifyEqual(tc, L(2).islimit(0), 0);
    verifyEqual(tc, L(2).islimit(3), 1);
    verifyEqual(tc, L(2).islimit(-2), -1);

%  isrevolute    true if joint is revolute
    verifyFalse(tc, L(1).isrevolute);
    verifyTrue(tc, L(2).isrevolute);
    
%  isprismatic   true if joint is prismatic
    verifyFalse(tc, L(2).isprismatic);
    verifyTrue(tc, L(1).isprismatic);
    
    syms a1 m1 c1 Iyy1 b1;
    L = Revolute('d', 0, 'a', a1, 'alpha', 0, 'm', m1, 'r', [c1 0 0], 'I', [0 Iyy1 0], 'B', b1, 'G', 1, 'Jm', 0, 'standard');
    L
    L.dyn
end

function deepcopy_test(tc)
    % deep copy test
    L = Link([1 2 3 4]);
    L2 = Link(L);
    verifyEqual(tc, L.a, 3);
    verifyEqual(tc, L2.a, 3);

    L.a = 10;
    verifyEqual(tc, L.a, 10);
    verifyEqual(tc, L2.a, 3);
end

function set_test(tc)
    L = Link([1 1 1 1 1]);

    L.r = [1 2 3];
    verifyEqual(tc, L.r, [1 2 3]);
    L.r = [1 2 3]';
    verifyEqual(tc, L.r, [1 2 3]);

    L.I = [1, 2, 3];
    verifyEqual(tc, L.I, diag([1,2,3]));
    L.I = [1, 2, 3, 4, 5, 6];
    verifyEqual(tc, L.I, [1 4 6; 4 2 5; 6 5 3]);
    A = rand(3,3);
    L.I = A'*A;     % make a symm matrix
    verifyEqual(tc, L.I, A'*A);

    L.Tc = 1;
    verifyEqual(tc, L.Tc, [1 -1]);
    L.Tc = [1 -2];
    verifyEqual(tc, L.Tc, [1 -2]);
    
    % all these should all fail
    function set_fail1
        L = Link;
        L.Tc = [-2 1];  % wrong order
    end
    function set_fail2
        L = Link;
        L.r = [ 1 2]';  % too short
    end
    
    function set_fail3
        L = Link;
        L.I = 1;  % too short
    end
    
    function set_fail4
        L = Link;
        I = diag([1 2 3]);
        I(1,2) = 1;
        L.I = I;        % not symmetric
    end
    
    verifyError(tc, @set_fail1, 'RTB:Link:badarg');
    verifyError(tc, @set_fail2, 'RTB:Link:badarg');
    verifyError(tc, @set_fail3, 'RTB:Link:badarg');
    verifyError(tc, @set_fail4, 'RTB:Link:badarg');

end
    
function viscous_friction_test(tc)
    L = Link();
    B = 1; G = 2; Tc = 0;

    % viscous model
    L.G = G;
    L.B = B;
    L.Tc = Tc;

    verifyEqual(tc, L.friction(1), -B*G^2);
    verifyEqual(tc, L.friction(-1), B*G^2);

    L.G = -G
    verifyEqual(tc, L.friction(1), -B*G^2);
    verifyEqual(tc, L.friction(-1), B*G^2);
end

function coulomb_friction_test(tc)
    L = Link();
    B = 0; G = 2; Tc = [2 -3];

    % coulomb model
    L.G = G
    L.B = 0;
    L.Tc = Tc;

    verifyEqual(tc, L.friction(1), -Tc(1)*G);
    verifyEqual(tc, L.friction(-1), -Tc(2)*G);

    % check for negative gear ratio
    L.G = -G

    % coulomb model
    L.B = 0;
    L.Tc = Tc;

    verifyEqual(tc, L.friction(1), -Tc(1)*G);
    verifyEqual(tc, L.friction(-1), -Tc(2)*G);
end


%  nofriction    remove joint friction
function nofriction_test(tc)
    L = Link();
    B = 1; G = 2; Tc = [2 -3];
    L.B = B;
    L.G = G
    L.Tc = Tc;

    verifyFalse(tc,  L.B == 0);
    verifyFalse(tc,  all(L.Tc == 0) );

    Lnf = L.nofriction('all');
    verifyTrue(tc,  Lnf.B == 0);
    verifyTrue(tc,  all(Lnf.Tc == 0) );
    % check original object unchanged
    verifyFalse(tc,  L.B == 0);
    verifyFalse(tc,  all(L.Tc == 0) );

    Lnf = L.nofriction('viscous');
    verifyTrue(tc,  Lnf.B == 0);
    verifyFalse(tc,  all(Lnf.Tc == 0) );

    Lnf = L.nofriction('coulomb');
    verifyFalse(tc,  Lnf.B == 0);
    verifyTrue(tc,  all(Lnf.Tc == 0) );
end

% test the convenience subclasses
function revolute_test(tc)
    L = Revolute();
    
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue( L.isrevolute );
    
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.d, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.mdh, 0);
    
    L = Revolute('d', 1, 'a', 2, 'alpha', 3, 'B', 4);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
end

function prismatic_test(tc)
    L = Prismatic();
    
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue( L.isprismatic );
    
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.theta, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.mdh, 0);
    
    L = Prismatic('theta', 1, 'a', 2, 'alpha', 3, 'B', 4);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
end

function revolute_mdh_test(tc)
    L = RevoluteMDH();
    
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue( L.isrevolute );
    
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.d, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.mdh, 1);
    
    L = RevoluteMDH('d', 1, 'a', 2, 'alpha', 3, 'B', 4);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
end

function prismatic_mdh_test(tc)
    L = PrismaticMDH();
    
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue( L.isprismatic );
    
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.theta, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.mdh, 1);
    
    L = PrismaticMDH('theta', 1, 'a', 2, 'alpha', 3, 'B', 4);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
end

