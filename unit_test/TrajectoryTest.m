%% This is for testing the Trajectory Generation functions in the robotics Toolbox
function tests = TrajectoryTest
  tests = functiontests(localfunctions);
  clc
end

function teardownOnce(tc)
    close all
end


function tpoly_test(tc)

    
    s1 = 1;
    s2 = 2;
    
    %% no boundary conditions
    
    tpoly(s1, s2, 11);
    
    s = tpoly(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    
    [s,sd] = tpoly(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(s >= 0));  % velocity is >= 0
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    [s,sd,sdd] = tpoly(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(sd >= -10*eps));  % velocity is >= 0
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    tc.verifyEqual(sdd(1), 0, 'absTol',1e-10);   % acceleration 
    tc.verifyEqual(sdd(6), 0, 'absTol',1e-10);
    tc.verifyEqual(sdd(end), 0, 'absTol',1e-10);
    tc.verifyEqual(sum(sdd), 0, 'absTol',1e-10);
    
    %% time vector version
    t = linspace(0, 1, 11);
    
    tpoly(s1, s2, t);
    
    s = tpoly(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    
    [s,sd] = tpoly(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(s >= 0));  % velocity is >= 0
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    [s,sd,sdd] = tpoly(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(sd >= 0));  % velocity is >= 0
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    tc.verifyEqual(sdd(1), 0, 'absTol',1e-10);   % acceleration 
    tc.verifyEqual(sdd(6), 0, 'absTol',1e-10);
    tc.verifyEqual(sdd(end), 0, 'absTol',1e-10);
    tc.verifyEqual(sum(sdd), 0, 'absTol',1e-10);

    %% boundary conditions
    [s,sd,sdd] = tpoly(s1, s2, 11, -1, 1);
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyEqual(sd(1), -1, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 1, 'absTol',1e-10);
    
    tc.verifyEqual(sdd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sdd(end), 0, 'absTol',1e-10);
    
    %% plot version
    tpoly(s1, s2, 11, -1, 1);
end
        
function lspb_test(tc)
    s1 = 1;
    s2 = 2;
    
    %% no boundary conditions
    lspb(s1, s2, 11);
    
    
    s = lspb(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    
    [s,sd] = lspb(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(s >= 0));  % velocity is >= 0
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    [s,sd,sdd] = lspb(s1, s2, 11);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(sd >= 0));  % velocity is >= 0
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    tc.verifyEqual(sum(sdd), 0, 'absTol',1e-10);

    %% time vector version
    t = linspace(0, 1, 11);
    lspb(s1, s2, t);
    
    s = lspb(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    
    [s,sd] = lspb(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(s >= 0));  % velocity is >= 0
    tc.verifyEqual(s(6), 1.5, 'absTol',1e-10);
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    [s,sd,sdd] = lspb(s1, s2, t);
    tc.verifyTrue(all(diff(s) > 0));  % output is monotonic
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyTrue(all(sd >= 0));  % velocity is >= 0
    tc.verifyEqual(sd(1), 0, 'absTol',1e-10);
    tc.verifyEqual(sd(end), 0, 'absTol',1e-10);
    
    tc.verifyEqual(sum(sdd), 0, 'absTol',1e-10);
        
    %% specify velocity
    [s,sd,sdd] = lspb(s1, s2, 11,0.2);
    tc.verifyEqual(s(1), s1, 'absTol',1e-10);
    tc.verifyEqual(s(end), s2, 'absTol',1e-10);
    
    tc.verifyEqual(sd(6), 0.2, 'absTol',1e-10);

    %% plot version
    lspb(s1, s2, 11);
end
        
%    ctraj                      - Cartesian trajectory
function ctraj_test(tc)
    % unit testing ctraj with T0 and T1 and N
    T0 = transl(1,2,3);
    T1 = transl(-1,-2,-3)*trotx(pi);
    
    T = ctraj(T0, T1, 3);
    tc.verifyEqual(size(T,3), 3);
    tc.verifyEqual(T(:,:,1), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), trotx(pi/2), 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);

    % unit testing ctraj with T0 and T1 and S(i)
    T = ctraj(T0, T1, [1 0 0.5]);
    tc.verifyEqual(size(T,3), 3);

    tc.verifyEqual(T(:,:,1), T1, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), trotx(pi/2), 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), T0, 'abstol', 1e-10);
end
            
%    jtraj                      - joint space trajectory
function jtraj_test(tc)
    % unit testing jtraj with 
    q1 = [1 2 3 4 5 6];
    q2 = -q1;
    
    q = jtraj(q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = jtraj(q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = jtraj(q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    tc.verifyEqual(qdd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qdd(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(qdd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    %% with a time vector
    t = linspace(0, 1, 11);
    
    q = jtraj(q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = jtraj(q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = jtraj(q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    tc.verifyEqual(qdd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qdd(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(qdd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    %% test with boundary conditions
    qone = ones(1,6);
    
    [q,qd,qdd] = jtraj(q1,q2,11, -qone, qone);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), -qone, 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), qone, 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    tc.verifyEqual(qdd(1,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(qdd(end,:), zeros(1,6), 'abstol', 1e-10);
    
end
     
         
%    mtraj                      - multi-axis trajectory for arbitrary function        
function mtraj_tpoly_test(tc)
    q1 = [1 2 3 4 5 6];
    q2 = -q1;
    
    q = mtraj(@tpoly, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = mtraj(@tpoly, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = mtraj(@tpoly, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    tc.verifyEqual(qdd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qdd(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(qdd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    %% with a time vector
    t = linspace(0, 1, 11);
    
    q = mtraj(@tpoly, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = mtraj(@tpoly, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = mtraj(@tpoly, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    tc.verifyEqual(qdd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qdd(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(qdd(end,:), zeros(1,6), 'abstol', 1e-10);
end

function mtraj_lspb_test(tc)
    q1 = [1 2 3 4 5 6];
    q2 = -q1;
    
    q = mtraj(@lspb, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = mtraj(@lspb, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = mtraj(@lspb, q1,q2,11);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    
    tc.verifyEqual(sum(qdd), zeros(1,6), 'abstol', 1e-10);
    
    %% with a time vector
    t = linspace(0, 1, 11);
    
    q = mtraj(@lspb, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd] = mtraj(@lspb, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    [q,qd,qdd] = mtraj(@lspb, q1,q2,t);
    tc.verifyEqual(size(q,1), 11);
    tc.verifyEqual(size(q,2), 6);
    tc.verifyEqual(q(1,:), q1, 'abstol', 1e-10);
    tc.verifyEqual(q(end,:), q2, 'abstol', 1e-10);
    tc.verifyEqual(q(6,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qd,1), 11);
    tc.verifyEqual(size(qd,2), 6);
    tc.verifyEqual(qd(1,:), zeros(1,6), 'abstol', 1e-10);
    tc.verifyEqual(qd(end,:), zeros(1,6), 'abstol', 1e-10);
    
    tc.verifyEqual(size(qdd,1), 11);
    tc.verifyEqual(size(qdd,2), 6);
    
    tc.verifyEqual(sum(qdd), zeros(1,6), 'abstol', 1e-10);
end

%    mstraj                     - multi-axis multi-segment trajectory
function mstraj_test(tc)
     via = [4 1; 4 4; 5 2; 2 5];
     %Test with QDMAX 
     out = mstraj(via, [ 2 1 ],[],[4 1],1,1);
     expected_out = [4.0000    1.0000
                     4.0000    1.7500
                     4.0000    2.5000
                     4.0000    3.2500
                     4.3333    3.3333
                     4.6667    2.6667
                     4.2500    2.7500
                     3.5000    3.5000
                     2.7500    4.2500
                     2.0000    5.0000];
     verifyEqual(tc, out,expected_out ,'absTol',1e-4);

     % test plotting
     mstraj(via, [ 2 1 ],[],[4 1],1,1);

     %Test with QO 
     out = mstraj(via, [],[2 1 3 4],[4 1],1,1);
     expected_out = [4.0000    1.0000
                     4.0000    4.0000
                     4.3333    3.3333
                     4.6667    2.6667
                     4.2500    2.7500
                     3.5000    3.5000
                     2.7500    4.2500
                     2.0000    5.0000];
     verifyEqual(tc, out,expected_out ,'absTol',1e-4);
     %Test with QO 
     out = mstraj(via, [],[1 2 3 4],via(1,:),1,1);
     expected_out = [4.0000    1.0000
                     4.0000    2.5000
                     4.3333    3.3333
                     4.6667    2.6667
                     4.2500    2.7500
                     3.5000    3.5000
                     2.7500    4.2500
                     2.0000    5.0000];
     verifyEqual(tc, out,expected_out ,'absTol',1e-4);

     [out,t] = mstraj(via, [],[1 2 3 4],via(1,:),1,1);
     verifyEqual(tc, size(t,1), size(out,1));
     verifyEqual(tc, size(t,2), 1);
     
     [out,t,info] = mstraj(via, [],[1 2 3 4],via(1,:),1,1);
     verifyEqual(tc, size(t,1), size(out,1));
     verifyEqual(tc, size(t,2), 1);
     
     verifyEqual(tc, length(info), size(via,1)+1);
     verifyTrue(tc, isa(info, 'struct'));
end

%    qplot                      - plot joint angle trajectories
function qplot_test(tc)
    s = linspace(0, 1, 100)';
    qz = [0 0 0 0 0 0]; qr = [1 2 3 4 5 6];
    q = (1-s)*qz + s*qr;
    clf
    qplot(s*20,q);

    qplot(q);
end

