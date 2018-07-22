%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox

function tests = PluckerTest
  tests = functiontests(localfunctions);
  
  clc
end

function teardownOnce(tc)
    close all
end

% Primitives
function constructor1_test(tc)
    
    % construct from 6-vector
    L = Plucker([1 2 3 4 5 6]);
    tc.verifyTrue( isa(L, 'Plucker') );
    tc.verifyEqual(L.v, [1 2 3]','absTol',1e-10);
    tc.verifyEqual(L.w, [4 5 6]','absTol',1e-10);
    
    % construct from object
    L2 = Plucker(L)
    tc.verifyTrue( isa(L, 'Plucker') );
    tc.verifyEqual(L2.v, [1 2 3]','absTol',1e-10);
    tc.verifyEqual(L2.w, [4 5 6]','absTol',1e-10);
    
    % construct from 2 3-vectors
    L = Plucker('wv', [1 2 3], [4 5 6]);
    tc.verifyEqual(L.w, [1 2 3]','absTol',1e-10);
    tc.verifyEqual(L.v, [4 5 6]','absTol',1e-10);
    tc.verifyEqual(L.uw, unit([1 2 3])','absTol',1e-10);
end

function double_test(tc)
    % verify double
    L = Plucker('wv', [1 2 3], [4 5 6]);
    tc.verifyEqual(double(L), [4 5 6 1 2 3]','absTol',1e-10);
end

function constructor2_test(tc)
    % 2 point constructor
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    tc.verifyEqual(L.w', P-Q,'absTol',1e-10);
    tc.verifyEqual(L.v', cross(P-Q,Q),'absTol',1e-10);

    % test all possible input shapes
    L2 = Plucker(P', Q);
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    L2 = Plucker(P, Q');
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    L2 = Plucker(P', Q');
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    L2 = Plucker('points', P, Q);
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    
    % planes constructor
    P = [10 11 12]'; w = [1 2 3];
    L = Plucker('Pw', P, w);
    tc.verifyEqual(double(L), [cross(w,P) w]','absTol',1e-10); %FAIL
    L2 = Plucker('Pw', P', w);
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    L2 = Plucker('Pw', P, w');
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
    L2 = Plucker('Pw', P', w');
    tc.verifyEqual(double(L2), double(L), 'absTol',1e-10);
end

function pp_test(tc)
    % validate pp and ppd
    L = Plucker([-1 1 2], [1 1 2]);
    tc.verifyEqual(L.pp, [0 1 2]', 'absTol',1e-10);
    tc.verifyEqual(L.ppd, sqrt(5), 'absTol',1e-10);
    
    % validate pp
    tc.verifyTrue( L.contains(L.pp) );
end

function contains_test(tc)
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    % validate contains
    tc.verifyTrue( L.contains([2 3 7]) );
    tc.verifyTrue( L.contains([2 1 0]) );
    tc.verifyFalse( L.contains([2 1 2]) );
end


function closest_test(tc)
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    [p,d] = L.closest(P);
    tc.verifyEqual(p, P', 'absTol',1e-10);
    tc.verifyEqual(d, 0, 'absTol',1e-10);
    
     % validate closest with given points and origin
    [p,d] = L.closest(Q);
    tc.verifyEqual(p, Q', 'absTol',1e-10);
    tc.verifyEqual(d, 0, 'absTol',1e-10);
    
    L = Plucker([-1 1 2], [1 1 2]);
    [p,d] = L.closest([0 1 2]);
    tc.verifyEqual(p, [0 1 2]', 'absTol',1e-10);
    tc.verifyEqual(d, 0, 'absTol',1e-10);
    
    [p,d] = L.closest([5 1 2]);
    tc.verifyEqual(p, [5 1 2]', 'absTol',1e-10);
    tc.verifyEqual(d, 0, 'absTol',1e-10);
    
    [p,d] = L.closest([0 0 0]);
    tc.verifyEqual(p, L.pp, 'absTol',1e-10);
    tc.verifyEqual(d, L.ppd, 'absTol',1e-10);
    
    [p,d] = L.closest([5 1 0]);
    tc.verifyEqual(p, [5 1 2]', 'absTol',1e-10);
    tc.verifyEqual(d, 2, 'absTol',1e-10);
end

function plot_test(tc)
    
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    axis(10*[-1 1 -1 1 -1 1]);
    plot(L, 'r', 'LineWidth', 2);
end

function eq_test(tc)
    w = [1 2 3];
    P = [-2 4 3];
    
    L1 = Plucker(P, P+w);
    L2 = Plucker(P+2*w, P+5*w);
    L3 = Plucker(P+[1 0 0], P+w);
    
    tc.verifyTrue(L1 == L2);
    tc.verifyFalse(L1 == L3);
    
    tc.verifyFalse(L1 ~= L2);
    tc.verifyTrue(L1 ~= L3);
end

function skew_test(tc)
    
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    m = L.skew;
    
    tc.verifyEqual(size(m), [4 4]);
    tc.verifyEqual(m+m', zeros(4,4), 'abstol', 1e-10);  
end

function mtimes_test(tc)
    P = [1 2 0]; Q = [1 2 10];  % vertical line through (1,2)
    L = Plucker(P, Q);
    
    % check pre/post multiply by matrix
    M = rand(4,10);
    
    a = L * M;
    tc.verifyEqual(a, L.skew*M);
    
    M = rand(10,4);
    a = M * L;
    tc.verifyEqual(a, M*L.skew);
    
    % check transformation by SE3
    
    L2 = SE3 * L;
    tc.verifyEqual(L.double, L2.double);
    
    L2 = SE3(2, 3, 1) * L; % shift line in the xy directions
    pxy = L2.intersect_plane([0 0 1 0]);
    tc.verifyEqual(pxy, [1+2 2+3 0]');
    
end

function parallel_test(tc)
    
    L1 = Plucker('Pw', [4 5 6], [1 2 3]);
    L2 = Plucker('Pw', [5 5 6], [1 2 3]);
    L3 = Plucker('Pw', [4 5 6], [3 2 1]);
    
    % L1 || L2 but doesnt intersect
    % L1 intersects L3
    
    tc.verifyTrue( isparallel(L1, L1) );
    tc.verifyTrue(L1 | L1);
    
    tc.verifyTrue( isparallel(L1, L2) );
    tc.verifyTrue(L1 | L2);
    tc.verifyTrue( isparallel(L2, L1) );
    tc.verifyTrue(L2 | L1);
    tc.verifyFalse( isparallel(L1, L3) );
    tc.verifyFalse(L1 | L3);
end

function intersect_test(tc)

    
    L1 = Plucker('Pw', [4 5 6], [1 2 3]);
    L2 = Plucker('Pw', [5 5 6], [1 2 3]);
    L3 = Plucker('Pw', [4 5 6], [0 0 1]);
    L4 = Plucker('Pw', [5 5 6], [1 0 0]);
    
    % L1 || L2 but doesnt intersect
    % L3 intersects L4
    tc.verifyFalse( L1^L2 );
    
    tc.verifyTrue( L3^L4 );
   
end

function commonperp_test(tc)
    L1 = Plucker('Pw', [4 5 6], [0 0 1]);
    L2 = Plucker('Pw', [6 5 6], [0 1 0]);
    
    tc.verifyFalse( L1|L2 );
    tc.verifyFalse( L1^L2 );
    
    tc.verifyEqual( distance(L1, L2), 2);
    
    L = commonperp(L1, L2);  % common perp intersects both lines
    
    tc.verifyTrue( L^L1 );
    tc.verifyTrue( L^L2 );
end


function line_test(tc)
    
    % mindist
    % intersect
    % char
    % intersect_volume
    % mindist
    % mtimes
    % or
    % side
end

function point_test(tc)
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    tc.verifyTrue( L.contains(L.point(0)) );
    tc.verifyTrue( L.contains(L.point(1)) );
    tc.verifyTrue( L.contains(L.point(-1)) );
end

function char_test(tc)
    P = [2 3 7]; Q = [2 1 0];
    L = Plucker(P, Q);
    
    s = char(L);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 1);
    
    s = char([L L]);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 2);
    s = char([L L]');
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 2);
end

function plane_test(tc)
    
    xyplane = [0 0 1 0];
    xzplane = [0 1 0 0];
    L = Plucker('planes', xyplane, xzplane); % x axis
    tc.verifyEqual(double(L), [0 0 0 -1 0 0]','absTol',1e-10);
    
    L = Plucker([-1 2 3], [1 2 3]);  %line at y=2,z=3
    x6 = [1 0 0 -6]; % x = 6
    
    % plane_intersect
    [p,lambda] = L.intersect_plane(x6)
    tc.verifyEqual(p, [6 2 3]','absTol',1e-10);
    tc.verifyEqual(L.point(lambda), [6 2 3]','absTol',1e-10);
    
    x6s.n = [1 0 0];
    x6s.p = [6 0 0];
    [p,lambda] = L.intersect_plane(x6s)
    tc.verifyEqual(p, [6 2 3]','absTol',1e-10);
    tc.verifyEqual(L.point(lambda), [6 2 3]','absTol',1e-10);
end