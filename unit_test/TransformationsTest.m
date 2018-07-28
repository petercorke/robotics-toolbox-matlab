%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox

function tests = TransformationsTest
  tests = functiontests(localfunctions);
  
  clc
end

function teardownOnce(tc)
    close all
end
% Primitives
function rotx_test(tc)
    verifyEqual(tc, rotx(0), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, rotx(pi/2), [1 0 0; 0 0 -1; 0 1 0],'absTol',1e-10);
    verifyEqual(tc, rotx(pi), [1 0 0; 0 -1 0; 0 0 -1],'absTol',1e-10);

    verifyEqual(tc, rotx(90, 'deg'), [1 0 0; 0 0 -1; 0 1 0],'absTol',1e-10);
    verifyEqual(tc, rotx(180, 'deg'), [1 0 0; 0 -1 0; 0 0 -1],'absTol',1e-10);
    
    syms q
    R = rotx(q);
    verifyInstanceOf(tc, R, 'sym');
    verifySize(tc, R, [3 3]);
    verifyEqual(tc, simplify(det(R)), sym(1));

    %test for non-scalar input
    verifyError(tc, @()rotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
function roty_test(tc)
    verifyEqual(tc, roty(0), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, roty(pi/2), [0 0 1; 0 1 0; -1 0 0],'absTol',1e-10);
    verifyEqual(tc, roty(pi), [-1 0 0; 0 1 0; 0 0 -1],'absTol',1e-10);

    verifyEqual(tc, roty(90, 'deg'), [0 0 1; 0 1 0; -1 0 0],'absTol',1e-10);
    verifyEqual(tc, roty(180, 'deg'), [-1 0 0; 0 1 0; 0 0 -1],'absTol',1e-10);
    
    syms q
    R = roty(q);
    verifyInstanceOf(tc, R, 'sym');
    verifySize(tc, R, [3 3]);
    verifyEqual(tc, simplify(det(R)), sym(1));
    
    %test for non-scalar input
    verifyError(tc, @()roty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
function rotz_test(tc)
    verifyEqual(tc, rotz(0), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, rotz(pi/2), [0 -1 0; 1 0 0; 0 0 1],'absTol',1e-10);
    verifyEqual(tc, rotz(pi), [-1 0 0; 0 -1 0; 0 0 1],'absTol',1e-10);

    verifyEqual(tc, rotz(90, 'deg'), [0 -1 0; 1 0 0; 0 0 1],'absTol',1e-10);
    verifyEqual(tc, rotz(180, 'deg'), [-1 0 0; 0 -1 0; 0 0 1],'absTol',1e-10);
    
    syms q
    R = rotz(q);
    verifyInstanceOf(tc, R, 'sym');
    verifySize(tc,R, [3 3]);
    verifyEqual(tc, simplify(det(R)), sym(1));
    
     %test for non-scalar input
    verifyError(tc, @()rotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
 
function trotx_test(tc)
    verifyEqual(tc, trotx(0), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, trotx(pi/2), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, trotx(pi), [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-10);

    verifyEqual(tc, trotx(90, 'deg'), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, trotx(180, 'deg'), [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-10);

    %test for non-scalar input
    verifyError(tc, @()trotx([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end
    
function troty_test(tc)
    verifyEqual(tc, troty(0), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, troty(pi/2), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, troty(pi), [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-10);

    verifyEqual(tc, troty(90, 'deg'), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, troty(180, 'deg'), [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-10);
     %test for non-scalar input
    verifyError(tc, @()troty([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end
    
function trotz_test(tc)
    verifyEqual(tc, trotz(0), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, trotz(pi/2), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, trotz(pi), [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-10);

    verifyEqual(tc, trotz(90, 'deg'), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-10);
    verifyEqual(tc, trotz(180, 'deg'), [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-10);
     %test for non-scalar input
    verifyError(tc, @()trotz([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end

function transl_test(tc)
    
    % transl(P) -> T
    verifyEqual(tc, transl(1, 2, 3), [1 0 0 1; 0 1 0 2; 0 0 1 3; 0 0 0 1], 'AbsTol', 1e-10);
     verifyEqual(tc, transl([1, 2, 3]), [1 0 0 1; 0 1 0 2; 0 0 1 3; 0 0 0 1], 'AbsTol', 1e-10);
     
     x = rand(4,3);
     T = transl(x);
     verifyEqual(tc, T(1:3,4,1), x(1,:)', 'AbsTol', 1e-10);
     verifyEqual(tc, T(1:3,4,4), x(4,:)', 'AbsTol', 1e-10);
         
     % transl(T) -> P
     verifyEqual(tc, transl([1 0 0 1; 0 1 0 2; 0 0 1 3; 0 0 0 1]), [1 2 3]', 'AbsTol', 1e-10);
     [a,b,c] = transl([1 0 0 1; 0 1 0 2; 0 0 1 3; 0 0 0 1]);
     verifyEqual(tc, a, 1);
     verifyEqual(tc, b, 2);
     verifyEqual(tc, c, 3);
     
     verifyEqual(tc, transl(T), x, 'AbsTol', 1e-10);
end
   

%% SO(2)
function rot2_test(tc)
    verifyEqual(tc,  rot2(0), eye(2,2), 'absTol', 1e-6);
    verifyEqual(tc,  trot2(0), eye(3,3), 'absTol', 1e-6);
    verifyEqual(tc,  rot2(0), eye(2,2), 'absTol', 1e-6);
    verifyEqual(tc,  trot2(pi/2), [0 -1 0; 1 0 0; 0 0 1], 'absTol', 1e-6);
    
    verifyEqual(tc,  trot2(90, 'deg'),[0 -1 0; 1 0 0; 0 0 1], 'absTol', 1e-6);
    verifyEqual(tc,  trot2(pi)*trot2(-pi/2), ...
        trot2(pi/2), 'absTol', 1e-6);
    verifyEqual(tc,  rot2(pi)*rot2(-pi/2), ...
        rot2(pi/2), 'absTol', 1e-6);
end

%% SE(2)
function SE2_test(tc)
    
    T2 = [1 0 1; 0 1 2; 0 0 1];
    
    % transl(T) -> P
    verifyEqual(tc,  transl2(T2), ...
        [1;2], 'absTol',1e-10);
    
    % transl(P) -> T
    verifyEqual(tc,  transl2(1, 2), ...
        [1 0 1; 0 1 2; 0 0 1], 'absTol', 1e-6);
    verifyEqual(tc,  transl2([2, 3]), ...
        [1 0 2; 0 1 3; 0 0 1], 'absTol', 1e-6);
    
    T3 = transl2([1 2; 3 4; 5 6]);
    verifySize(tc, T3, [3 3 3]);
    verifyEqual(tc, T3(:,:,2), transl2([3 4]))
    
    
    T2f = [1 1 1; 1 1 2; 0 0 1];
    R2 = [1 0 ; 0 1];
    R2f = [1 1 ; 1 1];
    verifyEqual(tc,  ishomog2(T2), true);
    verifyEqual(tc,  ishomog2(T2,1), true);
    verifyEqual(tc,  ishomog2(T2f,1), false);
    verifyEqual(tc,  ishomog2(R2), false);

    verifyEqual(tc,  isrot2(R2), true);
    verifyEqual(tc,  isrot2(R2,1), true);
    verifyEqual(tc,  isrot2(R2f,1), false);
    verifyEqual(tc,  isrot2(T2), false);
    
    
%     verifyEqual(tc,  SE2(2, 3, 0), ...
%         [1 0 2; 0 1 3; 0 0 1], 'absTol', 1e-6);
%     verifyEqual(tc,  SE2(2, 3, pi/2), ...
%         transl2(2,3)*trot2(pi/2), 'absTol', 1e-6);
end



%% angle/vector form
%    angvec2r                   - angle/vector to RM
function angvec2r_test(tc)
    
    verifyEqual(tc, angvec2r( pi/2, [1 0 0]), rotx(pi/2),'absTol',1e-10);
    verifyEqual(tc, angvec2r( pi/2, [0 1 0]), roty(pi/2),'absTol',1e-10);
    verifyEqual(tc, angvec2r( pi/2, [0 0 1]), rotz(pi/2),'absTol',1e-10);
    
    verifyEqual(tc, angvec2r(0, [1 0 0]), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, angvec2r(0, [0 1 0]), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, angvec2r(0, [0 0 1]), eye(3,3),'absTol',1e-10);
    verifyEqual(tc, angvec2r(0, [0 0 0]), eye(3,3),'absTol',1e-10);
    
    verifyError(tc, @()angvec2r(1, [0 0 0]),'RTB:angvec2r:badarg');

    verifyError(tc, @()angvec2r([1,2,3],0.1),'RTB:angvec2r:badarg');
    verifyError(tc, @()angvec2r(1),'RTB:angvec2r:badarg');
end

%    angvec2tr                  - angle/vector to HT
function angvec2tr_test(tc)
        
    verifyEqual(tc, angvec2tr( pi/2, [1 0 0]), trotx(pi/2),'absTol',1e-10);
    verifyEqual(tc, angvec2tr( pi/2, [0 1 0]), troty(pi/2),'absTol',1e-10);
    verifyEqual(tc, angvec2tr( pi/2, [0 0 1]), trotz(pi/2),'absTol',1e-10);
    
    verifyEqual(tc, angvec2tr(0, [1 0 0]), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, angvec2tr(0, [0 1 0]), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, angvec2tr(0, [0 0 1]), eye(4,4),'absTol',1e-10);
    verifyEqual(tc, angvec2tr(0, [0 0 0]), eye(4,4),'absTol',1e-10);
    
    verifyError(tc, @()angvec2tr(1, [0 0 0]),'RTB:angvec2r:badarg');
    
    verifyError(tc, @()angvec2tr([1,2,3],0.1),'RTB:angvec2r:badarg');
    verifyError(tc, @()angvec2tr(1),'RTB:angvec2tr:badarg');
end
     
%    tr2angvec                  - HT/RM to angle/vector form
function tr2angvec_test(tc)
	% null rotation
    % - vector isn't defined here, but RTB sets it (0 0 0)
    [theta, v] = tr2angvec(eye(3,3));
    verifyEqual(tc, theta, 0.0, 'absTol',1e-6);
    verifyEqual(tc, v, [0 0 0], 'absTol',1e-6);
    
    tr2angvec(eye(3,3))
    
    % canonic rotations
    [theta, v] = tr2angvec(rotx(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [1 0 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(roty(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [0 1 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(rotz(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [0 0 1], 'absTol',1e-6);
    
    % null rotation
    [theta, v] = tr2angvec(eye(4,4));
    verifyEqual(tc, theta, 0.0, 'absTol',1e-6);
    verifyEqual(tc, v, [0 0 0], 'absTol',1e-6);
    
    % canonic rotations
    [theta, v] = tr2angvec(trotx(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [1 0 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(troty(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [0 1 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(trotz(pi/2));
    verifyEqual(tc, theta, pi/2, 'absTol',1e-6);
    verifyEqual(tc, v, [0 0 1], 'absTol',1e-6);
     
    [theta, v] = tr2angvec(roty(pi/2), 'deg');
    verifyEqual(tc, theta, 90, 'absTol',1e-6);
    verifyEqual(tc, v, [0 1 0], 'absTol',1e-6);
    
    R = cat(3, rotx(pi/2), roty(pi/2), rotz(pi/2));
    [theta, v] = tr2angvec(R);
    verifyEqual(tc, theta, pi/2*[1 1 1]', 'absTol',1e-6);
    verifyEqual(tc, v, eye(3,3), 'absTol',1e-6);
    
    T = cat(3, trotx(pi/2), troty(pi/2), trotz(pi/2));
    [theta, v] = tr2angvec(T);
    verifyEqual(tc, theta, pi/2*[1 1 1]', 'absTol',1e-6);
    verifyEqual(tc, v, eye(3,3), 'absTol',1e-6);
    
    %test for scalar input
    verifyError(tc, @()tr2angvec(1), 'RTB:tr2angvec:badarg');
end
    


%% 3-angle forms

function eul2r_test(tc)
    
    % ZYZ
    r2d = 180/pi;
    
    R = rotz(0.1) * roty(0.2) * rotz(0.3);
    
    verifyEqual(tc, eul2r(0.1, 0.2, 0.3), R, 'absTol',1e-10); 
    verifyEqual(tc, eul2r([0.1, 0.2, 0.3]), R, 'absTol',1e-10);
    verifyEqual(tc, eul2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), R, 'absTol',1e-10); 
    verifyEqual(tc, eul2r([0.1, 0.2, 0.3]*r2d, 'deg'), R, 'absTol',1e-10);
    
    % trajectory case
    Rs = eul2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    Rs = eul2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);

    %test for scalar input
    verifyError(tc, @()eul2r(1),'RTB:eul2r:badarg');
end
    
%    eul2tr                     - Euler angles to HT
function eul2tr_test(tc)
    r2d = 180/pi;
    
    T = trotz(0.1) * troty(0.2) * trotz(0.3);
    
    verifyEqual(tc, eul2tr(0.1, 0.2, 0.3), T, 'absTol',1e-10); 
    verifyEqual(tc, eul2tr([0.1, 0.2, 0.3]), T, 'absTol',1e-10);
    verifyEqual(tc, eul2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), T, 'absTol',1e-10); 
    verifyEqual(tc, eul2tr([0.1, 0.2, 0.3]*r2d, 'deg'), T, 'absTol',1e-10);
    
    % trajectory case
    Ts = eul2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);
    
    Ts = eul2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);

    %test for scalar input
    verifyError(tc, @()eul2tr(1),'RTB:eul2r:badarg');
end

function rpy2r_test(tc)
    
    r2d = 180/pi;
    
    %% default zyx order
    R = rotz(0.3) * roty(0.2) * rotx(0.1);
    
    verifyEqual(tc, rpy2r(0.1, 0.2, 0.3), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3]), R, 'absTol',1e-10);
    verifyEqual(tc, rpy2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3]*r2d, 'deg'), R, 'absTol',1e-10);
    
    % trajectory case
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);

    %% xyz order

    R = rotx(0.3) * roty(0.2) * rotz(0.1);
    
    verifyEqual(tc, rpy2r(0.1, 0.2, 0.3, 'xyz'), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3], 'xyz'), R, 'absTol',1e-10);
    verifyEqual(tc, rpy2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', 'xyz'), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3]*r2d, 'deg', 'xyz'), R, 'absTol',1e-10);  
    
    % trajectory case
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3], 'xyz');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'xyz', 'deg');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    %% yxz order

    R = roty(0.3) * rotx(0.2) * rotz(0.1);
    
    verifyEqual(tc, rpy2r(0.1, 0.2, 0.3, 'yxz'), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3], 'yxz'), R, 'absTol',1e-10);
    verifyEqual(tc, rpy2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', 'yxz'), R, 'absTol',1e-10); 
    verifyEqual(tc, rpy2r([0.1, 0.2, 0.3]*r2d, 'deg', 'yxz'), R, 'absTol',1e-10);  
    
    % trajectory case
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3], 'yxz');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'yxz', 'deg');
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), R, 'absTol',1e-10);
    
    %test for scalar input
    verifyError(tc, @()rpy2tr(1),'RTB:rpy2r:badarg');
end

function rpy2tr_test(tc)

    r2d = 180/pi;
    
    T = trotz(0.3) * troty(0.2) * trotx(0.1);
    seq = 'zyx';
    verifyEqual(tc, rpy2tr(0.1, 0.2, 0.3, seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3], seq), T, 'absTol',1e-10);
    verifyEqual(tc, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg', seq), T, 'absTol',1e-10);
    

    T = trotx(0.3) * troty(0.2) * trotz(0.1);
    seq = 'xyz';
    verifyEqual(tc, rpy2tr(0.1, 0.2, 0.3, seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3], seq), T, 'absTol',1e-10);
    verifyEqual(tc, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg', seq), T, 'absTol',1e-10);
    
    T = troty(0.3) * trotx(0.2) * trotz(0.1);
    seq = 'yxz';
    verifyEqual(tc, rpy2tr(0.1, 0.2, 0.3, seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3], seq), T, 'absTol',1e-10);
    verifyEqual(tc, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', seq), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg', seq), T, 'absTol',1e-10);
    
    
    % trajectory case
    T = trotz(0.3) * troty(0.2) * trotx(0.1);

    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);
    
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);

    T = trotx(0.3) * troty(0.2) * trotz(0.1);
    
    verifyEqual(tc, rpy2tr(0.1, 0.2, 0.3, 'xyz'), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3], 'xyz'), T, 'absTol',1e-10);
    verifyEqual(tc, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', 'xyz'), T, 'absTol',1e-10); 
    verifyEqual(tc, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg', 'xyz'), T, 'absTol',1e-10);  
    
    % trajectory case
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3], 'xyz');
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);
    
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'xyz', 'deg');
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), T, 'absTol',1e-10);
    
    %test for scalar input
    verifyError(tc, @()rpy2tr(1),'RTB:rpy2r:badarg');
end

function tr2eul_test(tc)

    eul = [0.1 0.2 0.3];
    R = eul2r(eul);
    verifyEqual(tc, tr2eul(R), eul,'absTol',1e-10);
    verifyEqual(tc, tr2eul(R, 'deg'), eul*180/pi,'absTol',1e-10);

    Rs = cat(3, R, R, R, R);
    x = tr2eul(Rs);
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), eul,'absTol',1e-10);
    x = tr2eul(Rs, 'deg');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), eul*180/pi,'absTol',1e-10);

    T = eul2tr(eul);
    verifyEqual(tc, tr2eul(T), eul,'absTol',1e-10);
    verifyEqual(tc, tr2eul(T, 'deg'), eul*180/pi,'absTol',1e-10);

    Ts = cat(3, T, T, T, T);
    x = tr2eul(Ts);
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), eul,'absTol',1e-10);
    x = tr2eul(Ts, 'deg');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), eul*180/pi,'absTol',1e-10);
    
    % test singularity case
    eul = [0.1 0 0.3];
    R = eul2r(eul);
    verifyEqual(tc, eul2r( tr2eul(R) ), R,'absTol',1e-10);
    verifyEqual(tc, eul2r( tr2eul(R, 'deg'), 'deg'), R,'absTol',1e-10);

    %test for scalar input
    verifyError(tc, @()tr2eul(1),'RTB:SO3:check:badarg');
end
    
function tr2rpy_test(tc)
    rpy = [0.1 0.2 0.3];
    R = rpy2r(rpy);
    verifyEqual(tc, tr2rpy(R), rpy,'absTol',1e-10);
    verifyEqual(tc, tr2rpy(R, 'deg'), rpy*180/pi,'absTol',1e-10);

    Rs = cat(3, R, R, R, R);
    x = tr2rpy(Rs);
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy,'absTol',1e-10);
    x = tr2rpy(Rs, 'deg');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy*180/pi,'absTol',1e-10);

    T = rpy2tr(rpy);
    verifyEqual(tc, tr2rpy(T), rpy,'absTol',1e-10);
    verifyEqual(tc, tr2rpy(T, 'deg'), rpy*180/pi,'absTol',1e-10);

    Ts = cat(3, T, T, T, T);
    x = tr2rpy(Ts);
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy,'absTol',1e-10);
    x = tr2rpy(Ts, 'deg');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy*180/pi,'absTol',1e-10);

    % xyz order
    R = rpy2r(rpy, 'xyz');
    verifyEqual(tc, tr2rpy(R, 'xyz'), rpy,'absTol',1e-10);
    verifyEqual(tc, tr2rpy(R, 'deg', 'xyz'), rpy*180/pi,'absTol',1e-10);

    Rs = cat(3, R, R, R, R);
    x = tr2rpy(Rs, 'xyz');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy,'absTol',1e-10);
    x = tr2rpy(Rs, 'deg', 'xyz');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy*180/pi,'absTol',1e-10);

    T = rpy2tr(rpy, 'xyz');
    verifyEqual(tc, tr2rpy(T, 'xyz'), rpy,'absTol',1e-10);
    verifyEqual(tc, tr2rpy(T, 'deg', 'xyz'), rpy*180/pi,'absTol',1e-10);

    Ts = cat(3, T, T, T, T);
    x = tr2rpy(Ts, 'xyz');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy,'absTol',1e-10);
    x = tr2rpy(Ts, 'deg', 'xyz');
    verifySize(tc, x, [4 3]);
    verifyEqual(tc, x(2,:), rpy*180/pi,'absTol',1e-10);
    
    % corner cases
    seq = 'zyx';
    ang = [pi 0 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 pi 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 0 pi];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 pi/2 0]; % singularity
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 -pi/2 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);

    seq = 'xyz';
    ang = [pi 0 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 pi 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 0 pi];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 pi/2 0]; % singularity
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 -pi/2 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    
    seq = 'yxz';
    ang = [pi 0 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 pi 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 0 pi];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
      ang = [0 pi/2 0]; % singularity
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);
    ang = [0 -pi/2 0];
    a = rpy2tr(ang, seq);
    verifyEqual(tc, rpy2tr(tr2rpy(a, seq), seq), a, 'absTol',1e-10);

    %test for scalar input
    verifyError(tc, @()tr2rpy(1),'RTB:tr2rpy:badarg');
end
    
        

%    oa2r                       - orientation and approach vector to RM
function oa2r_test(tc)
    %Unit test for oa2r with variables ([0 1 0] & [0 0 1]) 
    verifyEqual(tc, oa2r([0 1 0], [0 0 1]),...
        [1     0     0
         0     1     0
         0     0     1],'absTol',1e-10);
    %test for scalar input
    verifyError(tc, @()oa2r(1),'RTB:oa2r:badarg');
end

%    oa2tr                      - orientation and approach vector to HT
function oa2tr_test(tc)
    %Unit test for oa2tr with variables ([0 1 0] & [0 0 1]) 
    verifyEqual(tc, oa2tr([0 1 0], [0 0 1]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absTol',1e-10);
    %test for scalar input
    verifyError(tc, @()oa2tr(1),'RTB:oa2tr:badarg');
end
    
%    r2t                        - RM to HT
function r2t_test(tc)
    %Unit test for r2t

    % SO(3) case
    R = [1 2 3;4 5 6; 7 8 9];
    verifyEqual(tc, r2t(R),...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absTol',1e-10);

    % sequence case
    Rs = cat(3, R, R, R);
    Ts = r2t(Rs);
    verifySize(tc, Ts, [4 4 3]);
    verifyEqual(tc, Ts(:,:,2), ...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absTol',1e-10);

    % SO(2) case
    R = [1 2; 3 4];
    verifyEqual(tc, r2t(R),...
        [1 2 0; 3 4 0; 0 0 1],'absTol',1e-10);
end
   
%    t2r                        - HT to RM
function t2r_test(tc)
    %Unit test for r2t with variables eul2tr([.1, .2, .3])

    % SE(3) case
    T = [1 2 3 4; 5 6 7 8; 9 10 11 12; 0 0 0 1];
    verifyEqual(tc, t2r(T),...
        [1 2 3; 5 6 7; 9 10 11],'absTol',1e-10);

    % sequence case
    Ts = cat(3, T, T, T);
    Rs = t2r(Ts);
    verifySize(tc, Rs, [3 3 3]);
    verifyEqual(tc, Rs(:,:,2), ...
        [1 2 3; 5 6 7; 9 10 11],'absTol',1e-10);

    % SE(2) case
    T = [1 2 3; 4 5 6; 0 0 1];
    verifyEqual(tc, t2r(T),...
        [1 2; 4 5],'absTol',1e-10);
end

%    tr2rt                        - HT to RM
function tr2rt_test(tc)
    %Unit test for r2t with variables eul2tr([.1, .2, .3])

    %% SE(3) case
    T = [1 2 3 4; 5 6 7 8; 9 10 11 12; 0 0 0 1];
    [R,t] = tr2rt(T);
    verifyEqual(tc, R, [1 2 3; 5 6 7; 9 10 11], 'absTol',1e-10);
    verifyEqual(tc, t, [4; 8; 12], 'absTol',1e-10);

    % sequence case
    Ts = cat(3, T, T, T, T, T);
    [Rs,ts] = tr2rt(Ts);
    verifySize(tc, Rs, [3 3 5]);
    verifySize(tc, ts, [5 3]);

    verifyEqual(tc, Rs(:,:,2), [1 2 3; 5 6 7; 9 10 11], 'absTol',1e-10);
    verifyEqual(tc, ts(2,:), [4 8 12], 'absTol',1e-10);

    %% SE(2) case
    T = [1 2 3; 4 5 6; 0 0 1];
    [R,t] = tr2rt(T);
    verifyEqual(tc, R, [1 2; 4 5], 'absTol',1e-10);
    verifyEqual(tc, t, [3;6], 'absTol',1e-10);
    
    Ts = cat(3, T, T, T, T, T);
    [Rs,ts] = tr2rt(Ts);
    verifySize(tc, Rs, [2 2 5]);
    verifySize(tc, ts, [5 2]);

    verifyEqual(tc, Rs(:,:,2), [1 2; 4 5], 'absTol',1e-10);
    verifyEqual(tc, ts(2,:), [3 6], 'absTol',1e-10);

end

function trchain_test(tc)
    a1 = 0;
    
    T = trchain('Tx(a1) Ty(a1) Tz(a1) Rx(a1) Ry(a1) Rz(a1)');
    tc.verifyEqual(T, eye(4,4), 'abstol', 1e-10);
    
    a1 = 1; a2 = 2; a3 = 3;
    tc.verifyEqual( trchain('Tx(a1) Ty(a2) Tz(a3)'), transl(1,2,3), 'abstol', 1e-10);
    
    a1 = 0.3; a2 = 0.4; a3 = 0.5;
    tc.verifyEqual( trchain('Rx(a1) Ry(a2) Rz(a3)'), trotx(0.3)*troty(0.4)*trotz(0.5), 'abstol', 1e-10);
    
    tc.verifyEqual( trchain('Rx(q1) Ry(q2) Rz(q3)', [.3,.4,.5]), trotx(0.3)*troty(0.4)*trotz(0.5), 'abstol', 1e-10);
    
    syms q1 q2 q3 a1 a2 a3
    tc.verifyEqual( trchain('Rx(q1) Tx(a1) Ry(q2) Ty(a2) Rz(q3) Tz(a3)', [q1 q2 q3]), trotx(q1)*transl(a1,0,0)*troty(q2)*transl(0,a2,0)*trotz(q3)*transl(0,0,a3) );
end


function trchain2_test(tc)
    a1 = 0;
    
    T = trchain2('Tx(a1) Ty(a1) R(a1)');
    tc.verifyEqual(T, eye(3,3), 'abstol', 1e-10);
    
    a1 = 1; a2 = 2;
    tc.verifyEqual( trchain2('Tx(a1) Ty(a2)'), transl2(1,2), 'abstol', 1e-10);
    
    a1 = 0.3; a2 = 0.4; 
    % R() is the same as Rz()
    tc.verifyEqual( trchain2('R(a1) Rz(a2)'), trot2(0.3)*trot2(0.4), 'abstol', 1e-10);
    
    syms q1 a1 a2
    tc.verifyEqual( trchain2('R(q1) Tx(a1) Ty(a2)', [q1]), trot2(q1)*transl2(a1,0)*transl2(0,a2) );
end

function trinterp_test(tc)
    %% between two transforms
    T0 = transl(1,2,3);
    T1 = transl(-1,-2,-3)*trotx(pi);
    Tm = trotx(pi/2);
    
    T = trinterp(T0, T1, 0);
    tc.verifyEqual(size(T), [4 4]);
    tc.verifyEqual(T, T0, 'abstol', 1e-10);
    
    tc.verifyEqual(trinterp(T0, T1, 1), T1, 'abstol', 1e-10);
    tc.verifyEqual(trinterp(T0, T1, 0.5), Tm, 'abstol', 1e-10);
    
    T = trinterp(T0, T1, [0.5 0 1]);
    tc.verifyEqual(size(T), [4 4 3]);
    tc.verifyEqual(T(:,:,1), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);
    
    T = trinterp(T0, T1, 3);   % interpolate in 3 steps
    tc.verifyEqual(size(T), [4 4 3]);
    tc.verifyEqual(T(:,:,1), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);

    %% between identity and transform
    T0 = eye(4,4);
    T1 = transl(2,4,6)*trotx(pi);
    Tm = transl(1,2,3)*trotx(pi/2);
    
    T = trinterp(T1, 0);
    tc.verifyEqual(size(T), [4 4]);
    tc.verifyEqual(T, T0, 'abstol', 1e-10);
    
    tc.verifyEqual(trinterp(T1, 1), T1, 'abstol', 1e-10);
    tc.verifyEqual(trinterp(T1, 0.5), Tm, 'abstol', 1e-10);
    
    T = trinterp(T1, [0.5 0 1]);
    tc.verifyEqual(size(T), [4 4 3]);
    tc.verifyEqual(T(:,:,1), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);
    
    T = trinterp(T1, 3);   % interpolate in 3 steps
    tc.verifyEqual(size(T), [4 4 3]);
    tc.verifyEqual(T(:,:,1), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);
end
 

function trinterp2_test(tc)
    %% between two transforms
    T0 = transl2(1,2);
    T1 = transl2(-1,-2)*trot2(pi);
    Tm = trot2(pi/2);
    
    T = trinterp2(T0, T1, 0);
    tc.verifyEqual(size(T), [3 3]);
    tc.verifyEqual(T, T0, 'abstol', 1e-10);
    
    tc.verifyEqual(trinterp2(T0, T1, 1), T1, 'abstol', 1e-10);
    tc.verifyEqual(trinterp2(T0, T1, 0.5), Tm, 'abstol', 1e-10);
    
    T = trinterp2(T0, T1, [0.5 0 1]);
    tc.verifyEqual(size(T), [3 3 3]);
    tc.verifyEqual(T(:,:,1), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);
    
    T = trinterp2(T0, T1, 3);
    tc.verifyEqual(size(T), [3 3 3]);
    tc.verifyEqual(T(:,:,1), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);

    %% between identity and transform
    T0 = eye(3, 3);
    T1 = transl2(2,4)*trot2(pi);
    Tm = transl2(1,2)*trot2(pi/2);
    
    T = trinterp2(T1, 0);
    tc.verifyEqual(size(T), [3 3]);
    tc.verifyEqual(T, T0, 'abstol', 1e-10);
    
    tc.verifyEqual(trinterp2(T0, T1, 1), T1, 'abstol', 1e-10);
    tc.verifyEqual(trinterp2(T0, T1, 0.5), Tm, 'abstol', 1e-10);
    
    T = trinterp2(T0, T1, [0.5 0 1]);
    tc.verifyEqual(size(T), [3 3 3]);
    tc.verifyEqual(T(:,:,1), Tm, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,2), T0, 'abstol', 1e-10);
    tc.verifyEqual(T(:,:,3), T1, 'abstol', 1e-10);
end

     
%    trnorm                     - normalize HT
function trnorm_test(tc)
        
    R = [0.9 0 0; .2 .6 .3; .1 .2 .4]';
    verifyEqual(tc, det(trnorm(R)), 1, 'absTol', 1e-14);

    t = [1 2 3]';
    T = rt2tr(R, t);
    Tn = trnorm(T);
    verifyEqual(tc, det(trnorm(t2r(Tn))), 1, 'absTol', 1e-14);
    verifyEqual(tc, Tn(1:3,4), t);
    
    % vector input
    RR = cat(3, R, R, R, R);
    RRn = trnorm(RR);
    verifySize(tc, RRn, [3 3 4]);
    verifyEqual(tc, det(RRn(:,:,1)), 1, 'absTol', 1e-14);
        verifyEqual(tc, det(RRn(:,:,1)), 1, 'absTol', 1e-14);

%HACK)    verifyEqual(tc, arrayfun( @(x) det(trnorm(x)), RR

    %test for scalar input
    verifyError(tc, @()trnorm(1),'RTB:trnorm:badarg');    
end

function trprint_test(tc)

    a = transl([1,2,3]) * eul2tr([.1, .2, .3]);

    trprint(a);
    
    s = evalc( 'trprint(a)' );
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 1);
    s = evalc( 'trprint(cat(3, a, a, a))' );
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 1);
    
    trprint(a, 'euler');
    trprint(a, 'euler', 'radian');
    trprint(a, 'rpy');
    trprint(a, 'rpy', 'radian');
    trprint(a, 'rpy', 'radian', 'xyz');
    trprint(a, 'rpy', 'radian', 'zyx');
    trprint(a, 'rpy', 'radian', 'yxz');
    
    trprint(a, 'angvec');
    trprint(a, 'angvec', 'radian');
    trprint(a, 'angvec', 'radian', 'fmt', '%g');
    trprint(a, 'angvec', 'radian', 'fmt', '%g', 'label', 'bob');
    
    % vector case
    
    a = cat(3, a, a, a);
    trprint(a);
    
    s = evalc( 'trprint(a)' );
    tc.verifyTrue(isa(s, 'char') );
    tc.verifyEqual( length(regexp(s, '\n', 'match')), 4);
    
    trprint(a, 'euler');
    trprint(a, 'euler', 'radian');
    trprint(a, 'rpy');
    trprint(a, 'rpy', 'radian');
    trprint(a, 'rpy', 'radian', 'xyz');
    trprint(a, 'rpy', 'radian', 'zyx');
    trprint(a, 'rpy', 'radian', 'yxz');
    
    trprint(a, 'angvec');
    trprint(a, 'angvec', 'radian');
    trprint(a, 'angvec', 'radian', 'fmt', '%g');
    trprint(a, 'angvec', 'radian', 'fmt', '%g', 'label', 'bob');
end

function trprint2_test(tc)
    a = transl2([1,2]) * trot2(0.3);

    trprint2(a);
    
    s = evalc( 'trprint2(a)' );
    tc.verifyTrue(isa(s, 'char') );
    
    trprint2(a, 'radian');
    trprint2(a, 'fmt', '%g');
    trprint2(a, 'label', 'bob');
    
    a = cat(3, a, a, a);
    trprint2(a);
    
    s = evalc( 'trprint(a)' );
    tc.verifyTrue(isa(s, 'char') );
    tc.verifyEqual( length(regexp(s, '\n', 'match')), 4);
    
    trprint2(a, 'radian');
    trprint2(a, 'fmt', '%g');
    trprint2(a, 'label', 'bob');
end

function trscale_test(tc)

    tc.verifyEqual( trscale(1), eye(4,4) );
    tc.verifyEqual( trscale([1 2 3]), diag([1 2 3 1]) );
    tc.verifyEqual( trscale(1, 2, 3), diag([1 2 3 1]) );

end  

function wtrans_test(tc)
    
    v = [1 2 3 4 5 6]';
    
    tc.verifyEqual( wtrans(eye(4,4), v), v, 'abstol', 1e-10 );
    tc.verifyEqual( wtrans(trotx(pi/2), v), [1 3 -2 4 6 -5]', 'abstol', 1e-10 );       
    tc.verifyEqual( wtrans(troty(pi/2), v), [-3 2 1 -6 5 4]', 'abstol', 1e-10 );       
    tc.verifyEqual( wtrans(trotz(pi/2), v), [2 -1 3 5 -4 6]', 'abstol', 1e-10 ); 
end

function trlog_test(tc)
    %unit tests for matrix expon stuff

    %%% SO(3) tests
    % zero rotation case
    verifyEqual(tc, trlog( eye(3,3) ), skew([0 0 0]), 'absTol', 1e-6);

    % rotation by pi case
    verifyEqual(tc, trlog( rotx(pi) ), skew([pi 0 0]), 'absTol', 1e-6);
    verifyEqual(tc, trlog( roty(pi) ), skew([0 pi 0]), 'absTol', 1e-6);
    verifyEqual(tc, trlog( rotz(pi) ), skew([0 0 pi]), 'absTol', 1e-6);

    % general case
    verifyEqual(tc, trlog( rotx(0.2) ), skew([0.2 0 0]), 'absTol', 1e-6);
    verifyEqual(tc, trlog( roty(0.3) ), skew([0 0.3 0]), 'absTol', 1e-6);
    verifyEqual(tc, trlog( rotz(0.4) ), skew([0 0 0.4]), 'absTol', 1e-6);

    %%% SE(3) tests

    % pure translation
    verifyEqual(tc, trlog( transl([1 2 3]) ), ...
        [0 0 0 1; 0 0 0 2; 0 0 0 3; 0 0 0 0], 'absTol', 1e-6);

    % mixture
    T = transl([1 2 3])*trotx(0.3);
    verifyEqual(tc, trlog(T), logm(T), 'absTol', 1e-6);
    
    T = transl([1 2 3])*troty(0.3);
    verifyEqual(tc, trlog(T), logm(T), 'absTol', 1e-6);
    
    verifyError(tc, @()trlog(0),'RTB:trlog:badarg');
end


function trexp_test(tc)
    %unit tests for matrix log stuff

    %%% SO(3) tests
    
    %% so(3)
    
    % zero rotation case
    verifyEqual(tc, trexp(skew([0 0 0])), eye(3,3), 'absTol', 1e-6);
    
    %% so(3), theta
    
    verifyEqual(tc, trexp(skew([0 0 0]), 1), eye(3,3), 'absTol', 1e-6);

    % rotation by pi case
    verifyEqual(tc, trexp(skew([pi 0 0])), rotx(pi), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 pi 0])), roty(pi), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 0 pi])), rotz(pi), 'absTol', 1e-6);

    % general case
    verifyEqual(tc, trexp(skew([0.2 0 0])), rotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 0.3 0])), roty(0.3), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 0 0.4])), rotz(0.4), 'absTol', 1e-6);
    
    verifyEqual(tc, trexp(skew([1 0 0]), 0.2), rotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 1 0]), 0.3), roty(0.3), 'absTol', 1e-6);
    verifyEqual(tc, trexp(skew([0 0 1]), 0.4), rotz(0.4), 'absTol', 1e-6);
    
    verifyEqual(tc, trexp([1 0 0], 0.2), rotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp([0 1 0], 0.3), roty(0.3), 'absTol', 1e-6);
    verifyEqual(tc, trexp([0 0 1], 0.4), rotz(0.4), 'absTol', 1e-6);
    
    verifyEqual(tc, trexp([1 0 0]*0.2), rotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp([0 1 0]*0.3), roty(0.3), 'absTol', 1e-6);
    verifyEqual(tc, trexp([0 0 1]*0.4), rotz(0.4), 'absTol', 1e-6);
    
    
    %%% SE(3) tests

    %% sigma = se(3)
    % pure translation
    verifyEqual(tc, trexp( skewa([1 2 3 0 0 0]) ), transl([1 2 3]), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 0 0.2 0 0]) ), trotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 0 0 0.3 0]) ), troty(0.3), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 0 0 0 0.4]) ), trotz(0.4), 'absTol', 1e-6);

    % mixture
    T = transl([1 2 3])*trotx(0.2)*troty(0.3)*trotz(0.4);
    verifyEqual(tc, trexp(logm(T)), T, 'absTol', 1e-6);
    
    %% twist vector
    verifyEqual(tc, trexp( double(Twist(T))), T, 'absTol', 1e-6);
    
    %% (sigma, theta)
    verifyEqual(tc, trexp( skewa([1 0 0 0 0 0]), 2), transl([2 0 0]), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 1 0 0 0 0]), 2), transl([0 2 0]), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 1 0 0 0]), 2), transl([0 0 2]), 'absTol', 1e-6);
    
    verifyEqual(tc, trexp( skewa([0 0 0 1 0 0]), 0.2), trotx(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 0 0 1 0]), 0.2), troty(0.2), 'absTol', 1e-6);
    verifyEqual(tc, trexp( skewa([0 0 0 0 0 1]), 0.2), trotz(0.2), 'absTol', 1e-6);

    
    %% (twist, theta)
    verifyEqual(tc, trexp(Twist('R', [1 0 0], [0 0 0]).S, 0.3), trotx(0.3), 'absTol', 1e-6);

    
    T = transl([1 2 3])*troty(0.3);
    verifyEqual(tc, trexp(logm(T)), T, 'absTol', 1e-6);
end



