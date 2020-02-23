%% This is for testing the SerialLink functions in the robotics Toolbox
function tests = SerialLinkTest
    
    tests = functiontests(localfunctions);
    clc
    
end

function setupOnce(tc)
    
    mdl_puma560;
    tc.TestData.p560 = p560;
    mdl_planar2
    tc.TestData.p2 = p2;
    mdl_puma560akb
    tc.TestData.p560m = p560m;
    mdl_stanford
    tc.TestData.stanf = stanf;
end

function teardownOnce(tc)
    close all
end


function SerialLink_test(tc)
    % test making a robot from links
    L(1)=Link([1 1 1 1 1]);
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
        'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    % test makin a robot from DH matrix
    DH = [pi/2 0 0 1; 0 2 0 0];
    R2 = SerialLink(DH,'name','Robot2');
    
end

function SerialLinkMDH_test(tc)
    % minimalistic test of modified DH functionality
    %  most code is common, need to exercise MDH code in Link and also for
    %  RNE
    
    mdl_puma560akb
    
    % qr is not set by the script, it clashes with the builtin function QR(tc)
    qz = [0 0 0 0 0 0];
    
    T = p560m.fkine(qz);
    J = p560m.jacobe(qz);
    tau = p560m.rne(qz, qz, qz);
end


function dyn_test(tc)
    
    mdl_puma560
    p560.dyn;
    
    mdl_puma560akb
    p560m.dyn;
end

%    *                          - compound two robots
function compound_test(tc)
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
function fkine_test(tc)
    
    % create simple RP robot we can reason about
    L(1)=Link([0 2 3 0 0]);
    L(2)=Link([0 0 0 0 1]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
        'base', eye(4,4), 'tool', eye(4,4) );
    
    
    tc.verifyTrue(isa( R1.fkine([0,0]), 'SE3') );
    
    tc.verifyEqual(R1.fkine([0,0]).T, transl(3,0,2), 'absTol',1e-6);
    tc.verifyEqual(R1.fkine([pi/2,0]).T, transl(0,3,2)*trotz(pi/2), 'absTol',1e-6);
    tc.verifyEqual(R1.fkine([0,1]).T, transl(3,0,3), 'absTol',1e-6);
    
    mdl_puma560
    
    T=p560.fkine( [0 0 0 0 0 0; 0 0.03 -0.0365 0 0 0] );
    tc.verifyEqual(length(T), 2, 'SE3' );;
end

function plot_test(tc)
    clf
    L(1)=Link([1 1 1 1 0]);
    L(1).qlim = [-5 5];
    L(2)=Link([0 1 0 1 0]);
    R1 = SerialLink(L,'name','robot1','comment', 'test robot','manufacturer', 'test',...
        'base', eye(4,4), 'tool', eye(4,4), 'offset', [1 1 0 0 0 0 ] );
    R1.plot([1 1]);
    close all
end

function plot_animate_test(tc)
    tc.assumeTrue(ispc || ismac);
    fname = fullfile(tempdir, 'puma.mp4');
    qt = jtraj(tc.TestData.p560.qz, tc.TestData.p560.qr, 50);
    tc.TestData.p560.plot(qt, 'movie', fname);
    tc.verifyTrue(exist(fname, 'file') == 2);
    delete(fname)
end

function plot3d_test(tc)
    clf
    tc.TestData.p560.plot3d( tc.TestData.p560.qn );
    close(gcf)
end


function teach_test(tc)
    tc.TestData.p560.teach();
    close all
    
    tc.TestData.p560.teach('eul');
    close all
    tc.TestData.p560.teach('eul', 'deg');
    close all
    tc.TestData.p560.teach('rpy');
    close all
    tc.TestData.p560.teach('rpy/xyz');
    close all
    tc.TestData.p560.teach('rpy/zyx');
    close all
    tc.TestData.p560.teach('approach');
    close all

    tc.TestData.p560.teach('callback', @(r, q) fprintf('hello') );
    close all
    
    tc.TestData.p2.teach();
    close all
    
    tc.TestData.p2.teach('2d');
    close all
    
  
    figure
    figure
    tc.TestData.p2.teach([0.2 0.3]);
    close all
    
    close all

end

%%-------------------------------------------------------------------------- 
%inverse kinematics for 6-axis arm with sph.wrist
function ikine6s_puma_test(tc)
    
    % puma case
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    T = p560.fkine(qn);
    qik = p560.ikine6s(T,'ru');
    tc.verifyEqual(p560.fkine(qik).T, T.T, 'absTol', 1e-6);
    tc.verifyTrue(qik(2) > 0);
    
    T = p560.fkine(qn);
    qik = p560.ikine6s(T,'ld');
    tc.verifyEqual(p560.fkine(qik).T, T.T, 'absTol', 1e-6);
    tc.verifyTrue(qik(2) < 0);
    
    
    % error handling
    mdl_puma560akb
    
    tc.verifyError( @() p560m.ikine6s(T), 'RTB:ikine:notsupported' );
    tc.verifyError( @() tc.TestData.p2.ikine6s(T), 'RTB:ikine:notsupported' );
    
end


function ikine6s_stanford_test(tc)
   
    % stanford arm case
    q = [pi/4 pi/4 0.3 pi/4 -pi/4 pi/4];
    T = tc.TestData.stanf.fkine(q);
    qik = tc.TestData.stanf.ikine6s(T);
    tc.verifyEqual(tc.TestData.stanf.fkine(qik).T, T.T, 'absTol', 1e-6);
    
end

function ikine6s_KR5_test(tc)
   
    % kr5 arm case
    mdl_KR5
    q = [0 pi/4 -pi 1 pi/4 0];
    T = KR5.fkine(q);
    qik = KR5.ikine6s(T);
    tc.verifyEqual(KR5.fkine(qik).T, T.T, 'absTol', 1e-6);

end

function ikine6s_IRB140_test(tc)

    tc.assumeTrue(false);  %HACK
    
    % nooffset type
    mdl_irb140
    q = [0 -3*pi/4 0 0 pi/4 0];
    T = irb140.fkine(q);
    qik = irb140.ikine6s(T);
    tc.verifyEqual(irb140.fkine(qik).T, T.T, 'absTol', 1e-6);
    

end

function ikine_test(tc)
    T = tc.TestData.p560.fkine(tc.TestData.p560.qn);
    qik = tc.TestData.p560.ikine(T, [0 0 3 0 0 0]);
    
    T2 = tc.TestData.p560.fkine(qik);
    tc.verifyEqual(T.T, T2.T,'absTol',1e-6);
end

function ikunc_optim_test(tc)
    T = tc.TestData.p560.fkine(tc.TestData.p560.qn);
    qik = tc.TestData.p560.ikunc(T);
    
    T2 = tc.TestData.p560.fkine(qik);
    tc.verifyEqual(T.T, T2.T,'absTol',1e-6);
end

function ikcon_optim_test(tc)
    
    tc.assumeTrue(false);  %HACK

    qn = [0 pi/4 -pi 0 pi/4 0];
    T = tc.TestData.p560.fkine(qn);
    qik = tc.TestData.p560.ikcon(T);
    
    T2 = tc.TestData.p560.fkine(qik);
    tc.verifyEqual(T.T, T2.T,'absTol',1e-6);
end

function ikine_sym_test(tc)
    % 2DOF test
    p2 = SerialLink(tc.TestData.p2); % clone it
    
    q = p2.ikine_sym(2);
    
    % is the solution sane
    tc.verifyLength(q, 2);
    tc.verifyTrue(iscell(q));
    tc.verifyTrue(isa(q{1}, 'sym'));
    tc.verifyLength(q{1}, 2);
    tc.verifyTrue(isa(q{2}, 'sym'));
    tc.verifyLength(q{1}, 2);
    
    % process the solutions
    q1 = subs(q{1}, {'tx', 'ty'}, {1,1});   % convert to numeric
    sol = eval(q1)';
    q2 = subs(q{2}, {'tx', 'ty', 'q1'}, {1,1, q1(1)});
    x = eval(q2);  % first solution
    sol(1,2) = x(1);
    q2 = subs(q{2}, {'tx', 'ty', 'q1'}, {1,1, q1(2)}); % second solution
    x = eval(q2);  
    sol(2,2) = x(1);

    % check the FK is good
    tc.verifyEqual(transl(tc.TestData.p2.fkine(sol(1,:))), [1 1 0]);
    tc.verifyEqual(transl(tc.TestData.p2.fkine(sol(2,:))), [1 1 0]);

    tc.verifyError( @() tc.TestData.p2.ikine_sym(4), 'RTB:ikine_sym:badarg');
end

function ikine_sym2_test(tc)
    tc.assumeTrue(false);  %HACK
    % 3DOF test
    
    % create robot arm with no offset (IRB140 style)
    robot = SerialLink([0 0.5 0 pi/2; 0 0 0.5 0; 0 0 0.5 0])
    test_sym_ik(robot)
    
    % create robot arm with offset (Puma560 style)
    robot = SerialLink(tc.TestData.p560.links(1:3));
    robot = SerialLink(robot); % clone it
    test_sym_ik(robot)
    
    function test_sym_ik(robot)
        % test all 8 solutions are good
        
        q = robot.ikine_sym(3);
        
        % process the solutions
        T = robot.fkine([0.2, 0.3, 0.4]); % choose joint angles
        
        pe = num2cell(T.t');
        
        q1 = subs(q{1}, {'tx', 'ty', 'tz'}, pe);   % convert to numeric
        
        qik = [];
        ss = [];
        
        for s1=1:2
            try
                sol1 = eval(q1(s1));
            catch
                fprintf('** q1(%d) eval failure\n', s1);
            end
            
            q2 = subs(q{2}, {'tx', 'ty', 'tz', 'q1'}, [pe q1(s1)]);
            
            for s2=1:2
                try
                    sol2 = eval(q2(s2));
                catch
                    fprintf('** q2(%d) eval failure\n', s2);
                    continue;
                end
                q3 = subs(q{3}, {'tx', 'ty', 'tz', 'q1', 'q2'}, [pe q1(s1), q2(s2)]);
                
                for s3=1:2
                    try
                        sol3 = eval(q3(s3));
                        qik = [qik; sol1 sol2 sol3];
                        ss = [ss; s1 s2 s3];
                    catch
                        fprintf('** q3(%d) eval failure\n', s3);
                    end
                end
            end
        end
        
        % check the FK is good, zero residual
        nn = [];
        for qq=qik'
            nn = [nn; norm(transl(robot.fkine(qq')-T))];
        end
        tc.verifyEqual(sum(nn), 0, 'absTol', 1e-6);
    end
end


%%--------------------------------------------------------------------------

function jacob0_test(tc)
    
    function J = jacob0_approx(robot, q, d)
        e = eye(robot.n);
        T0 = robot.fkine(q);
        R0 = t2r(T0);
        J = [];
        for i=1:robot.n
            Ji = (robot.fkine(q + e(i,:)*d) - T0) / d;
            J = [J [Ji(1:3,4); vex(Ji(1:3,1:3)*R0')]];
        end
    end
    % implictly tests jacobe
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = tc.TestData.p560.jacob0(qz);
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    tc.verifyEqual(tc.TestData.p560.jacob0(qz), jacob0_approx(tc.TestData.p560, qz, 1e-8), 'absTol', 1e-4);
    tc.verifyEqual(tc.TestData.p560.jacob0(qn), jacob0_approx(tc.TestData.p560, qn, 1e-8), 'absTol', 1e-4);
    
    expected_out = jacob0_approx(tc.TestData.p560, qn, 1e-8)
    out = tc.TestData.p560.jacob0(qn*180/pi, 'deg');
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    out = tc.TestData.p560.jacob0(qn, 'rpy');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'eul');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'exp');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'trans');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [3 6]);
    out = tc.TestData.p560.jacob0(qn, 'rot');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [3 6]);
    
    tc.verifyEqual(tc.TestData.p560.jacob0(qn), jacob0_approx(tc.TestData.p560, qn, 1e-8), 'absTol', 1e-4);
    
    out = tc.TestData.stanf.jacob0([0 0 0 0 0 0]);
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    tc.verifyEqual(tc.TestData.stanf.jacob0(qn), jacob0_approx(tc.TestData.stanf, qn, 1e-8), 'absTol', 1e-4);
    
end

function jacobe_test(tc)
    
    function J = jacobe_approx(robot, q, d)
        e = eye(robot.n);
        T0 = robot.fkine(q);
        R0 = t2r(T0);
        J = [];
        for i=1:robot.n
            Ji = (robot.fkine(q + e(i,:)*d) - T0) / d;
            J = [J [R0'*Ji(1:3,4); vex(R0'*Ji(1:3,1:3))]];
        end
    end
    

    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    
    out = tc.TestData.p560.jacobe(qz)
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    expected_out = jacobe_approx(tc.TestData.p560, qz, 1e-8)
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    out = tc.TestData.p560.jacobe(qz*180/pi, 'deg');
    tc.verifyEqual(out,expected_out,'absTol',1e-4);    
    
    out = tc.TestData.p560.jacobe(qn);
    expected_out = jacobe_approx(tc.TestData.p560, qn, 1e-8)

    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    out = tc.TestData.p560.jacobe(qn*180/pi, 'deg');
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    out = tc.TestData.p560.jacob0(qn, 'rpy');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'eul');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'exp');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [6 6]);
    
    out = tc.TestData.p560.jacob0(qn, 'trans');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [3 6]);
    out = tc.TestData.p560.jacob0(qn, 'rot');
    tc.verifyClass(out, 'double');
    tc.verifySize(out, [3 6]);
    
    tc.verifyWarning( @() tc.TestData.p560.jacobn(qn), 'RTB:SerialLink:deprecated');
end

function maniplty_test(tc)
    mdl_puma560;
    q = [0 pi/4 -pi 1 pi/4 0];
    tc.verifyEqual(p560.maniplty(q), 0.0786, 'absTol',1e-4);
    tc.verifyEqual(p560.maniplty(q, 'trans'), 0.1112, 'absTol',1e-4);
    tc.verifyEqual(p560.maniplty(q, 'rot'), 2.5936, 'absTol',1e-4);
    tc.verifyEqual(p560.maniplty(q, 'asada', 'trans'), 0.2733, 'absTol',1e-4);
end

function jacob_dot_test(tc)
    
    tc.assumeTrue(false);  %HACK

    function Jdqd = jacob_dot_approx(robot, q, qd, d)
        e = eye(robot.n);
        J0 = robot.jacob0(q);
        Jdqd = zeros(6,1);
        for i=1:robot.n
            Ji = (robot.jacob0(q + e(i,:)*d) - J0) / d;
            Jdqd = Jdqd + Ji * qd(i);
        end
    end
    
    q = rand(1,6);
    qd = rand(1,6);
    
    jacob_dot_approx(tc.TestData.p560, q, qd, 1e-8)*qd'
    tc.TestData.p560.jacob_dot(q, qd)
end


%%     Dynamics methods

function rne_test(tc)
    
    % use something simple we can reason about
    
    mdl_twolink
    g = twolink.gravity(3);
    
    qz = [0 0];  % need this to get around a MATLAB bug passing values from a script
    
    % different poses
    Q = twolink.rne([0 0], qz, qz, 'slow');
    tc.verifyEqual(Q, g*[2 0.5], 'absTol', 1e-6);
    
    Q = twolink.rne([0 pi/2], qz, qz);
    tc.verifyEqual(Q, g*[1.5 0], 'absTol', 1e-6);
    
    Q = twolink.rne([pi/2 0], qz, qz);
    tc.verifyEqual(Q, g*[0 0], 'absTol', 1e-6);
    
    % change gravity
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0]);
    tc.verifyEqual(Q, g*[0 0], 'absTol', 1e-6);
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0]');
    tc.verifyEqual(Q, g*[0 0], 'absTol', 1e-6);
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0]);
    tc.verifyEqual(Q, g*[0 0], 'absTol', 1e-6);
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 -g]);
    tc.verifyEqual(Q, g*[-2 -0.5], 'absTol', 1e-6);
    
    % add an external force in end-effector frame
    %  y-axis is vertically upward
    
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0], 'fext', [1 0 0 0 0 0]);
    tc.verifyEqual(Q, [0 0], 'absTol', 1e-6);
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0], 'fext', [0 1 0 0 0 0]);
    tc.verifyEqual(Q, [2 1], 'absTol', 1e-6);
    Q = twolink.rne([0 0], qz, qz, 'gravity', [0 0 0], 'fext', [0 0 1 0 0 0]);
    tc.verifyEqual(Q, [0 0], 'absTol', 1e-6);
    
    
    % test the [q qd qdd] case
    Q1 = twolink.rne([1 2], [3 4], [5 6]);
    Q2 = twolink.rne([1 2 3 4 5 6]);
    tc.verifyEqual(Q1, Q2, 'absTol', 1e-6);
    
    
    % test the matrix input case
    Q1 = twolink.rne([1 2], [3 4], [5 6]);
    Q2 = twolink.rne([5 6], [1 2], [3 4]);
    Q3 = twolink.rne([3 4], [5 6], [1 2]);
    
    Q4 = twolink.rne([1 2; 5 6; 3 4], [3 4; 1 2; 5 6], [5 6; 3 4; 1 2]);
    tc.verifyEqual([Q1; Q2; Q3], Q4, 'absTol', 1e-6);
    
    % probably should do a robot with a prismatic axis (or two)
    
end

function rne_mdh_test(tc)
    q = rand(1,6);
    tc.TestData.p560m.rne(q, q, q);
end


%        accel                  - forward dynamics
function accel_test(tc)
    qd = 0.5 * [1 1 1 1 1 1];
    qz = [0 1 0 0 2 0];
    Q = tc.TestData.p560.rne(tc.TestData.p560.qn, qz, qz);
    
    out = tc.TestData.p560.accel(qz, qd, Q);
    expected_out = [  -9.3397 4.9666 1.6095 -5.4305 5.9885 -2.1228]';
    
    tc.verifyEqual(out, expected_out,'absTol',1e-4);
    
    out = tc.TestData.p560.accel([qz, qd,Q]);
    tc.verifyEqual(out, expected_out,'absTol',1e-4);
    
    out = tc.TestData.p560.accel([qz, qd,Q]');
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    
    qd = [0.1 0.1 0.1 0.1 0.1 0.1;0.2 0.2 0.2 0.2 0.2 0.2];
    qz = [0 1 0 0 2 0;0 0.5 0 0 1 0];
    qd1 = [0.1 0.1 0.1 0.1 0.1 0.1];
    qd2 = [0.2 0.2 0.2 0.2 0.2 0.2];
    qz1 = [0 1 0 0 2 0];
    qz2 = [0 0.5 0 0 1 0];
    qn1 = [0 pi/4 -pi 1 pi/4 0];
    qn2 = [0 pi/2 -pi 1 pi/2 0];
    
    Q1 = tc.TestData.p560.rne(qn1,qz1,qz1);
    Q2 = tc.TestData.p560.rne(qn2,qz2,qz2);
    
    q3 = [Q1;Q2];
    
    out = tc.TestData.p560.accel(qz, qd,q3);
    expected_out = [
        -8.2760    5.8119    3.1487   -4.6392    6.9558   -1.6774
        -8.3467   -4.8514    6.0575   -4.9232    3.1244   -1.7861 ];
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    tc.verifyError( @() tc.TestData.p560.accel(), 'RTB:accel:badarg');
    tc.verifyError( @() tc.TestData.p560.accel( [ 1 2]), 'RTB:accel:badarg');
    tc.verifyError( @() tc.TestData.p560.accel( qz, qz, qd1), 'RTB:accel:badarg');
    tc.verifyError( @() tc.TestData.p560.accel( qz, qd1, qd1), 'RTB:accel:badarg');
    
    tc.verifyError( @() tc.TestData.p560.accel( [1 2], qd1, qd1), 'RTB:accel:badarg');
    tc.verifyError( @() tc.TestData.p560.accel( qz1, [1 2], qd1), 'RTB:accel:badarg');
    tc.verifyError( @() tc.TestData.p560.accel( qz1, qz1, [1 2]), 'RTB:accel:badarg');

    tc.verifyError( @() tc.TestData.p560.accel( [ 1 2]), 'RTB:accel:badarg');

end

%        cinertia               - Cartesian manipulator inertia matrix
function cinertia_test(tc)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.cinertia(qn);
    expected_out = [18.0741   -2.2763   -8.8446   -0.0283    0.7541   -0.4015
        -2.2763   11.2461    1.7238   -0.0750    0.2214   -0.4733
        -8.8446    1.7238   14.1135   -0.0300    0.7525   -0.4032
        -0.0283   -0.0750   -0.0300    0.1377   -0.0171    0.0492
        0.7541    0.2214    0.7525   -0.0171    0.4612   -0.2461
        -0.4015   -0.4733   -0.4032    0.0492   -0.2461    0.3457];
    tc.verifyEqual(out,expected_out,'absTol',1e-4);;
end


%        coriolis               - centripetal/coriolis torque
function coriolis_test(tc)
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
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
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
    
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
end


function gravload_test(tc)
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = tc.TestData.p560.gravload(qn);
    expected_out = [-0.0000 31.6334 6.0286 -0.0119 0.0218 0];
    tc.verifyEqual(out, expected_out, 'absTol',1e-4);
    

    qn = [0 pi/4 -pi 1 pi/4 0; 0 pi/2 -pi 1 pi/2 0];
    out = tc.TestData.p560.gravload(qn);
    expected_out = [-0.0000   31.6334    6.0286   -0.0119    0.0218         0
        0.0000    7.7198    8.7439   -0.0238   -0.0000         0];
    tc.verifyEqual(out, expected_out, 'absTol', 1e-4);
    
    % zero gravity
    out = tc.TestData.p560.gravload(tc.TestData.p560.qn, [0 0 0]);
    tc.verifyEqual(out, [0 0 0 0 0 0], 'absTol', 1e-4);
    
    tc.verifyError( @() tc.TestData.p560.gravload([1 2 3]), 'RTB:SerialLink:gravload:badarg' );
end

function inertia_test(tc)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.inertia(qn);
    expected_out = [3.6591   -0.4042    0.1013   -0.0021   -0.0015   -0.0000
        -0.4042    4.4128    0.3504   -0.0008    0.0017    0.0000
        0.1013    0.3504    0.9377   -0.0008    0.0008    0.0000
        -0.0021   -0.0008   -0.0008    0.1925    0.0000    0.0000
        -0.0015    0.0017    0.0008    0.0000    0.1713    0.0000
        -0.0000    0.0000    0.0000    0.0000    0.0000    0.1941];
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
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
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
end

function dynamic_component_test(tc)
    % test that inertial components 
    q = rand(1,6);
    qd = rand(1,6);
    qdd = rand(1,6);
    
    robot = tc.TestData.p560.nofriction('all');
    
    I = robot.inertia(q);
    C = robot.coriolis(q, qd);
    g = robot.gravload(q)';
    
    tau = robot.rne(q, qd, qdd)';
    
    tc.verifyEqual(norm(I*qdd'+C*qd'+g-tau), 0, 'absTol', 1e-8);
end


%        itorque                - inertia torque
function itorque_test(tc)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qdd = 0.5 * [1 1 1 1 1 1];
    out = p560.itorque(qn,qdd);
    expected_out = [1.6763    2.1799    0.6947    0.0944    0.0861    0.0971];
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    %        rne                    - inverse dynamics
end
function rne2_test(tc)
    mdl_puma560;
    qz = [0 1 0 0 2 0];
    qn = [0 pi/4 -pi 1 pi/4 0];
    out = p560.rne(qn,qz,qz);
    expected_out = [-0.9748   59.1306    5.9889   -0.0108    1.8872    0.0001];
    tc.verifyEqual(out,expected_out,'absTol',1e-4);
    
    pnf = p560.nofriction('all');
    
    q = rand(1,6);
    qd = rand(1,6);
    qdd = rand(1,6);
    
    tau1 = pnf.rne(q,qd,qdd);
    tau2 = pnf.inertia(q)*qdd' +pnf.coriolis(q,qd)*qd' + pnf.gravload(q)';
    tc.verifyEqual(tau1', tau2, 'absTol', 1e-6);
end

%        fdyn                   - forward dynamics
function fdyn_test(tc)
    mdl_puma560;
    qn = [0 pi/4 -pi 1 pi/4 0];
    qd = 0.5 * [1 1 1 1 1 1];
    qdo = [0 0 0 0 0 0];
    T = 0.0001;
    
    p560 = p560.nofriction();
    
    [TI,Q,QD] = p560.fdyn(T, @(r,t,q,qd) zeros(1,6), qn, qdo);
end

function nofriction_test(tc)
    mdl_puma560;
    verifyFalse(tc,  p560.links(3).B == 0);
    verifyFalse(tc,  all(p560.links(3).Tc == 0) );
    
    nf = p560.nofriction();
    verifyFalse(tc,  nf.links(3).B == 0);
    tc.verifyTrue( all(nf.links(3).Tc == 0) );
    
    mdl_puma560;
    nf = p560.nofriction('all');
    tc.verifyTrue( nf.links(3).B == 0);
    tc.verifyTrue( all(nf.links(3).Tc == 0) );
end

function jtraj_test(tc)
    mdl_puma560
    qz = [0 0 0 0 0 0];
    qr = [0 pi/2 -pi/2 0 0 0];
    T1 = p560.fkine(qz);
    T2 = p560.fkine(qr);
    qt = p560.jtraj(T1, T2, 50, 'r');
    tc.verifyEqual( size(qt), [50 6]);
    
    %tc.verifyEqual(qt(1,:), qz, 'abstol', 1e-10);
    %tc.verifyEqual(qt(end,:), qr, 'abstol', 1e-10);
end

function edit_test(tc)
    robot = SerialLink(tc.TestData.p560);
    
    robot.edit()
    
    table = findobj('Type', 'uitable');
    table.Data(1,2) = 7;
    
    button = findobj('String', 'Save')
    f = button.Callback{1};
    f(button, [], table);


    tc.verifyEqual(robot.links(1).d, 7);
    close(gcf)
    
    robot.edit('dyn')
    
    table = findobj('Type', 'uitable');
    table.Data(1,9) = 7;
    
    button = findobj('String', 'Save')
    f = button.Callback{1};
    f(button, [], table);


    tc.verifyEqual(robot.links(1).m, 7);
    close(gcf)
end

function ellipse_test(tc)
    tc.TestData.p560.plot( tc.TestData.p560.qn );
    tc.TestData.p560.fellipse( tc.TestData.p560.qn );
    tc.TestData.p560.fellipse( tc.TestData.p560.qn, 'trans' );
    tc.TestData.p560.fellipse( tc.TestData.p560.qn, 'rot' );
    
    clf
    tc.TestData.p560.fellipse( tc.TestData.p560.qn, '2d' );
    
    close(gcf)
    
    tc.TestData.p560.plot( tc.TestData.p560.qn );
    tc.TestData.p560.vellipse( tc.TestData.p560.qn );
    tc.TestData.p560.vellipse( tc.TestData.p560.qn, 'trans' );
    tc.TestData.p560.vellipse( tc.TestData.p560.qn, 'rot' );
    clf
    tc.TestData.p560.fellipse( tc.TestData.p560.qn, '2d' );
    
    close(gcf)
end

function dh_mdh_test(tc)

    tc.verifyTrue(tc.TestData.p560.isdh)
    tc.verifyFalse(tc.TestData.p560.ismdh)
    
    tc.verifyClass(tc.TestData.p560.isdh, 'logical')
    tc.verifyClass(tc.TestData.p560.ismdh, 'logical')

    tc.verifyFalse(tc.TestData.p560m.isdh)
    tc.verifyTrue(tc.TestData.p560m.ismdh)
    
    p560m = tc.TestData.p560.MDH;
    tc.verifyTrue(isa(p560m, 'SerialLink'));
    tc.verifyEqual(p560m.n, 6);
    tc.verifyTrue(p560m.ismdh);

    p560 = tc.TestData.p560m.DH;
    tc.verifyTrue(isa(p560, 'SerialLink'));
    tc.verifyEqual(p560.n, 6);
    tc.verifyTrue(p560.isdh);
    
    p560m = tc.TestData.p560.MDH;
    p560 = p560m.DH;  % should be the same robot
    
    T1 = tc.TestData.p560.fkine(tc.TestData.p560.qn);
    T2 = p560.fkine(tc.TestData.p560.qn);
    tc.verifyEqual(T1, T2);
    

end

function twists_test(tc)
    [tw,T0] = tc.TestData.p560.twists(tc.TestData.p560.qz);
    tc.verifyClass(tw, 'Twist');
    tc.verifyLength(tw, 6);
    tc.verifyClass(T0, 'SE3');
    tc.verifyLength(T0, 1);
    
    tc.verifyEqual( double(prod( [tw.exp(tc.TestData.p560.qn) T0] )), ...
        double(tc.TestData.p560.fkine(tc.TestData.p560.qn)), ...
        'absTol', 1e-6);
end

function predicates_test(tc)
    % isdh
    x = tc.TestData.p560.isdh
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    x = tc.TestData.p560m.isdh
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyFalse(x);
    
    % ismdh
    x = tc.TestData.p560.ismdh
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyFalse(x);
    x = tc.TestData.p560m.ismdh
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    
    % isspherical
    x = tc.TestData.p560.isspherical
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    x = tc.TestData.p560.isspherical
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    
    % isprismatic
    x = tc.TestData.stanf.isprismatic
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 6); tc.verifyEqual(x, logical([0 0 1 0 0 0]));
    % isrevolute
    x = tc.TestData.stanf.isrevolute
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 6); tc.verifyEqual(x, ~logical([0 0 1 0 0 0]));
    
    % issym
    x = tc.TestData.p560.issym
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyFalse(x);
    mdl_twolink_sym;
    x = twolink.issym
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    
    % isconfig
    x = tc.TestData.p560.isconfig('RRRRRR');
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    x = tc.TestData.p560.isconfig('RRPRRR');
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyFalse(x);
    x = tc.TestData.stanf.isconfig('RRRRRR');
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyFalse(x);
    x = tc.TestData.stanf.isconfig('RRPRRR');
    tc.verifyClass(x, 'logical'); tc.verifyLength(x, 1); tc.verifyTrue(x);
    
    % islimit
    x = tc.TestData.p560.islimit([0 0 0 0 0 0]);
    tc.verifyClass(x, 'double'); tc.verifySize(x, [6 2]);
    tc.verifyEqual(x, zeros(6,2));
    x = tc.TestData.p560.islimit([0 10 0 0 0 0]);
    tc.verifyClass(x, 'double'); tc.verifySize(x, [6 2]);
    tc.verifyEqual(x, [0 1 0 0 0 0]'*[1 1]);
end

function test_get(tc)
    x = tc.TestData.p560.d;
    tc.verifyClass(x, 'double'); tc.verifyLength(x, 6);
    tc.verifyEqual(x, [0  0  0.15005  0.4318  0  0], 'absTol', 1e-6);
    
    x = tc.TestData.p560.a;
    tc.verifyClass(x, 'double'); tc.verifyLength(x, 6);
    tc.verifyEqual(x, [0 0.4318 0.0203  0  0 0], 'absTol', 1e-6);
    
    x = tc.TestData.p560.alpha;
    tc.verifyClass(x, 'double'); tc.verifyLength(x, 6);
    tc.verifyEqual(x, [1 0 -1 1 -1 0]*pi/2, 'absTol', 1e-6);
    
    x = tc.TestData.p560.theta;
    tc.verifyClass(x, 'double'); tc.verifyLength(x, 0);
    
end

function mat2str_test(tc)
    mdl_twolink_sym;
    twolink
end

function rad_deg_test(tc)
    q = [1 2 3 4 5 6];
    
    q2 = tc.TestData.p560.todegrees(q);
    tc.verifyEqual(q*180/pi, q2);
    q2 = tc.TestData.p560.toradians(q);
    tc.verifyEqual(q*pi/180, q2);
    
    q2 = tc.TestData.stanf.todegrees(q);
    qq = q*180/pi;
    qq(3) = q(3);
    tc.verifyEqual(qq, q2);
    q2 = tc.TestData.stanf.toradians(q);
    qq = q*pi/180;
    qq(3) = q(3);
    tc.verifyEqual(qq, q2);
    

end

function trchain_test(tc)
    % TODO need to test return values here
    s = tc.TestData.p560.trchain();
    tc.verifyClass(s, 'char');

    s = tc.TestData.p560m.trchain();
    tc.verifyClass(s, 'char');
end

function jointdynamics_test(tc)
    jd = tc.TestData.p560.jointdynamics( zeros(1,6), zeros(1,6));
    
    tc.verifySize(jd, [1 6]);
    tc.verifyClass(jd, 'tf');
end
