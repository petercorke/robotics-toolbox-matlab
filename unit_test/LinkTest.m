%% This is for testing the Link functions in the robotics Toolbox
function tests = LinkTest
    tests = functiontests(localfunctions);
end


function constructor_classic_dh_test(tc)
    L = Link();
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 0);
    tc.verifyEqual(L.d, 0);
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Link([1 2 3 4]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Link([1 2 3 4 1]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Link([1 2 3 4 1 5]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 5);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Link([1 2 3 4 1 5], 'standard');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 5);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    verifyError(tc, @() Link(1), 'RTB:Link:badarg');
    verifyError(tc, @() Link([1 2 3]), 'RTB:Link:badarg');
    
    % dynamics  standard revolute
    L = Link([1 2 3 4 0 6:19 -20]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    tc.verifyEqual(L.m, 6);
    tc.verifyEqual(L.r, [7:9]);
    tc.verifyEqual(diag(L.I)', [10:12]);
    tc.verifyEqual(diag(L.I,1)', [13:14]);
    tc.verifyEqual(diag(L.I,2), 15);
    tc.verifyEqual(L.Jm, 16);
    tc.verifyEqual(L.G, 17);
    tc.verifyEqual(L.B, 18);
    tc.verifyEqual(L.Tc, [19 -20]);
    
    % dynamics standard prismatic
    L = Link([1 2 3 4 1 6:19 -20]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    tc.verifyEqual(L.m, 6);
    tc.verifyEqual(L.r, [7:9]);
    tc.verifyEqual(diag(L.I)', [10:12]);
    tc.verifyEqual(diag(L.I,1)', [13:14]);
    tc.verifyEqual(diag(L.I,2), 15);
    tc.verifyEqual(L.Jm, 16);
    tc.verifyEqual(L.G, 17);
    tc.verifyEqual(L.B, 18);
    tc.verifyEqual(L.Tc, [19 -20]);
    
    
end

function constructor_classic_mdh_test(tc)
    %     L = Link('modified');
    %     tc.verifyTrue( isa(L, 'Link') );
    %     tc.verifyEqual(L.theta, 0);
    %     tc.verifyEqual(L.d, 0);
    %     tc.verifyEqual(L.a, 0);
    %     tc.verifyEqual(L.alpha, 0);
    %     tc.verifyEqual(L.offset, 0);
    %     tc.verifyEqual(L.type, 'R');
    %     tc.verifyTrue(isrevolute(L));
    %     tc.verifyFalse(isprismatic(L));
    %     tc.verifyEqual(L.mdh, 1);
    
    L = Link([1 2 3 4], 'modified');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    L = Link([1 2 3 4 1], 'modified');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    L = Link([1 2 3 4 1 5], 'modified');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.d, 2);
    tc.verifyEqual(L.a, 3);
    tc.verifyEqual(L.alpha, 4);
    tc.verifyEqual(L.offset, 5);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
end

function constructor_test(tc)
    
    % standard revolute
    L = Link('d', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    % standard revolute
    L = Link('d', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31], 'revolute', 'standard');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    % standard prismatic
    L = Link('theta', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31], 'prismatic');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    % modified revolute
    L = Link('d', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31], 'modified');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    % modified prismatic
    L = Link('theta', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31], 'modified', 'prismatic');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    verifyError(tc, @() Link('theta', 1, 'd', 2), 'RTB:Link:badarg');
    
    % symbolic
    L = Link('d', 1, 'a', 2, 'alpha', 3, 'sym');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue(L.issym);
    
    tc.verifyTrue(isa(L.d, 'sym'));
    tc.verifyTrue(eval(L.d == 1));
    
    tc.verifyTrue(isa(L.a, 'sym'));
    tc.verifyTrue(eval(L.a == 2));
    
    tc.verifyTrue(isa(L.alpha, 'sym'));
    tc.verifyTrue(eval(L.alpha == 3));
    
    L = Link('theta', 1, 'a', 2, 'alpha', 3, 'sym');
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue(L.issym);
    
    tc.verifyTrue(isa(L.theta, 'sym'));
    tc.verifyTrue(eval(L.theta == 1));
    
    tc.verifyTrue(isa(L.a, 'sym'));
    tc.verifyTrue(eval(L.a == 2));
    
    tc.verifyTrue(isa(L.alpha, 'sym'));
    tc.verifyTrue(eval(L.alpha == 3));
    
end

%% test the convenience subclasses
function constructor_revolute_test(tc)
    L = Revolute();
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 0);
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Revolute('d', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    tc.verifyEqual(L.type, 'R');
    
end



function constructor_prismatic_test(tc)
    L = Prismatic();
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 0);
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 0);
    
    L = Prismatic('theta', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    
    tc.verifyEqual(L.mdh, 0);
    
end



function constructor_revolute_mdh_test(tc)
    L = RevoluteMDH();
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 0);
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'R');
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    L = RevoluteMDH('d', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.d, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyTrue(isrevolute(L));
    tc.verifyFalse(isprismatic(L));
    tc.verifyEqual(L.type, 'R');
    
    tc.verifyEqual(L.mdh, 1);
end

function constructor_prismatic_mdh_test(tc)
    L = PrismaticMDH();
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 0);
    tc.verifyEqual(L.a, 0);
    tc.verifyEqual(L.alpha, 0);
    tc.verifyEqual(L.offset, 0);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
    
    L = PrismaticMDH('theta', 1, 'a', 2, 'alpha', 3, 'B', 4, 'm', 5, 'G', 6, 'Jm', 7, 'Tc', [8 -9], ...
        'r', [10 11 12], 'I', [21:26], 'qlim', [30 31]);
    tc.verifyTrue( isa(L, 'Link') );
    tc.verifyFalse(L.issym);
    tc.verifyEqual(L.theta, 1);
    tc.verifyEqual(L.a, 2);
    tc.verifyEqual(L.alpha, 3);
    tc.verifyEqual(L.B, 4);
    tc.verifyEqual(L.m, 5);
    tc.verifyEqual(L.G, 6);
    tc.verifyEqual(L.Jm, 7);
    tc.verifyEqual(L.Tc, [8 -9]);
    tc.verifyEqual(L.r, [10 11 12]);
    tc.verifyEqual(diag(L.I)', [21:23]);
    tc.verifyEqual(diag(L.I,1)', [24:25]);
    tc.verifyEqual(diag(L.I,2), 26);
    tc.verifyEqual(L.qlim, [30 31]);
    tc.verifyEqual(L.type, 'P');
    tc.verifyFalse(isrevolute(L));
    tc.verifyTrue(isprismatic(L));
    tc.verifyEqual(L.mdh, 1);
end



%%  construct a robot link object
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

function RP_test(tc)
    L = Link();
    
    tc.verifyEqual(L.RP, 'R');
    tc.verifyWarning( @() L.RP, 'RTB:Link:deprecated');
end

function type_test(tc)
    L = [ Revolute() Prismatic()  Prismatic() Revolute()];
    tc.verifyEqual(L.type, 'RPPR');
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
    
    L.Tc = [];  % does nothing
    verifyEqual(tc, L.friction(1), -Tc(1)*G);
    verifyEqual(tc, L.friction(-1), -Tc(2)*G);
        
    %tc.verifyError( @() L.Tc = [1 2 3], 'RTB:Link:badarg');
    
    function wrapper()
        L.Tc = [1 2 3]
    end
    
    tc.verifyError( @wrapper, 'RTB:Link:badarg');
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

function flip_test(tc)
    L = Link();
    tc.verifyFalse(L.flip);
    L = Link('flip');
    tc.verifyTrue(L.flip);
end

function A_test(tc)
    L = Link([1 2 3 pi/2]);
    tc.verifyEqual(L.A(0).T, [1 0 0 3; 0 0 -1 0; 0 1 0 2; 0 0 0 1], 'AbsTol', 1e-10);
    tc.verifyEqual(L.A({0}).T, [1 0 0 3; 0 0 -1 0; 0 1 0 2; 0 0 0 1], 'AbsTol', 1e-10);
    
    
    A = [0 0 1 0; 1 0 0 3; 0 1 0 2; 0 0 0 1];
    
    tc.verifyEqual(L.A(pi/2).T, A, 'AbsTol', 1e-10);
    L.flip = true;
    tc.verifyEqual(L.A(-pi/2).T, A, 'AbsTol', 1e-10);
    
end

function char_test(tc)
    
    L = [ Revolute() Prismatic()  Prismatic() Revolute()];
    s = char(L);
    tc.verifyTrue(ischar(s));
    tc.verifyEqual(size(s,1), 4);
    
    s = char(L, true);
    tc.verifyTrue(ischar(s));
    tc.verifyEqual(size(s,1), 4);
    
    
        L = [ RevoluteMDH() PrismaticMDH()  PrismaticMDH() RevoluteMDH()];
    s = char(L);
    tc.verifyTrue(ischar(s));
    tc.verifyEqual(size(s,1), 4);
    
    s = char(L, true);
    tc.verifyTrue(ischar(s));
    tc.verifyEqual(size(s,1), 4);
end

function plus_test(tc)
    R = Revolute()+Prismatic()+Prismatic()+ Revolute();
    
    tc.verifyTrue(isa(R, 'SerialLink'));
    tc.verifyEqual(R.n, 4);
    tc.verifyEqual(R.config, 'RPPR');
end

