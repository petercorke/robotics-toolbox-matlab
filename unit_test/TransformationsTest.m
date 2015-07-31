%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox

function tests = TransformationsTest
  tests = functiontests(localfunctions);
end

% Primitives
function rotx_test(testCase)
    %Unit test for rotz
    verifyEqual(testCase, rotx(0), eye(3,3),'absTol',1e-4);
    verifyEqual(testCase, rotx(pi/2), [1 0 0; 0 0 -1; 0 1 0],'absTol',1e-4);
    verifyEqual(testCase, rotx(pi), [1 0 0; 0 -1 0; 0 0 -1],'absTol',1e-4);

    verifyEqual(testCase, rotx(90, 'deg'), [1 0 0; 0 0 -1; 0 1 0],'absTol',1e-4);
    verifyEqual(testCase, rotx(180, 'deg'), [1 0 0; 0 -1 0; 0 0 -1],'absTol',1e-4);

    %test for non-scalar input
    verifyError(testCase, @()rotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
function roty_test(testCase)
    %Unit test for roty
    verifyEqual(testCase, roty(0), eye(3,3),'absTol',1e-4);
    verifyEqual(testCase, roty(pi/2), [0 0 1; 0 1 0; -1 0 0],'absTol',1e-4);
    verifyEqual(testCase, roty(pi), [-1 0 0; 0 1 0; 0 0 -1],'absTol',1e-4);

    verifyEqual(testCase, roty(90, 'deg'), [0 0 1; 0 1 0; -1 0 0],'absTol',1e-4);
    verifyEqual(testCase, roty(180, 'deg'), [-1 0 0; 0 1 0; 0 0 -1],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()roty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
function rotz_test(testCase)
    %Unit test for rotz
    verifyEqual(testCase, rotz(0), eye(3,3),'absTol',1e-4);
    verifyEqual(testCase, rotz(pi/2), [0 -1 0; 1 0 0; 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, rotz(pi), [-1 0 0; 0 -1 0; 0 0 1],'absTol',1e-4);

    verifyEqual(testCase, rotz(90, 'deg'), [0 -1 0; 1 0 0; 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, rotz(180, 'deg'), [-1 0 0; 0 -1 0; 0 0 1],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()rotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
 
function trotx_test(testCase)
    %Unit test for trotz
    verifyEqual(testCase, trotx(0), eye(4,4),'absTol',1e-4);
    verifyEqual(testCase, trotx(pi/2), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, trotx(pi), [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-4);

    verifyEqual(testCase, trotx(90, 'deg'), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, trotx(180, 'deg'), [1 0 0 0; 0 -1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-4);

    %test for non-scalar input
    verifyError(testCase, @()trotx([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end
    
function troty_test(testCase)
    %Unit test for troty
    verifyEqual(testCase, troty(0), eye(4,4),'absTol',1e-4);
    verifyEqual(testCase, troty(pi/2), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, troty(pi), [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-4);

    verifyEqual(testCase, troty(90, 'deg'), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, troty(180, 'deg'), [-1 0 0 0; 0 1 0 0; 0 0 -1 0; 0 0 0 1],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()troty([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end
    
function trotz_test(testCase)
    %Unit test for trotz
    verifyEqual(testCase, trotz(0), eye(4,4),'absTol',1e-4);
    verifyEqual(testCase, trotz(pi/2), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, trotz(pi), [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-4);

    verifyEqual(testCase, trotz(90, 'deg'), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-4);
    verifyEqual(testCase, trotz(180, 'deg'), [-1 0 0 0; 0 -1 0 0; 0 0 1 0; 0 0 0 1],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()trotz([1 2 3; 0 0 0 1]),'MATLAB:catenate:dimensionMismatch');
end

function transl_test(testCase)
    %Unit test for transl with variables (0.1, 0.2, 0.3)
    verifyEqual(testCase, transl(0.1, 0.2, 0.3),...
        [1.0000         0         0    0.1000
              0    1.0000         0    0.2000
              0         0    1.0000    0.3000
              0         0         0    1.0000],'absTol',1e-4);
    %Unit test for transl with variables ([0.1, 0.2, 0.3])
    verifyEqual(testCase, transl([0.1, 0.2, 0.3]),...
        [1.0000         0         0    0.1000
              0    1.0000         0    0.2000
              0         0    1.0000    0.3000
              0         0         0    1.0000],'absTol',1e-4);
    %Unit test for transl with variables [0 0 0]
    verifyEqual(testCase, transl([0 0 0] ),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absTol',1e-4);
    %Unit test for transl with variable (1)
    verifyEqual(testCase, transl(1),...
        [1     0     0     1
         0     1     0     1
         0     0     1     1
         0     0     0     1],'absTol',1e-4);
end
   
%% SE(2)
function SE2_test(testCase)
    
        T2 = [1 0 1; 0 1 2; 0 0 1];
    verifyEqual(testCase,  transl2(T2), ...
         [1;2], 'absTol',1e-4);
    
    T2f = [1 1 1; 1 1 2; 0 0 1];
    R2 = [1 0 ; 0 1];
    R2f = [1 1 ; 1 1];
    verifyEqual(testCase,  ishomog2(T2), true);
    verifyEqual(testCase,  ishomog2(T2,1), true);
    verifyEqual(testCase,  ishomog2(T2f,1), false);
    verifyEqual(testCase,  ishomog2(R2), false);

    verifyEqual(testCase,  isrot2(R2), true);
    verifyEqual(testCase,  isrot2(R2,1), true);
    verifyEqual(testCase,  isrot2(R2f,1), false);
    verifyEqual(testCase,  isrot2(T2), false);
    
    verifyEqual(testCase,  transl2(1, 2), ...
        [1 0 1; 0 1 2; 0 0 1], 'absTol', 1e-6);
    verifyEqual(testCase,  transl2([2, 3]), ...
        [1 0 2; 0 1 3; 0 0 1], 'absTol', 1e-6);
    verifyEqual(testCase,  SE2(2, 3, 0), ...
        [1 0 2; 0 1 3; 0 0 1], 'absTol', 1e-6);
    verifyEqual(testCase,  SE2(2, 3, pi/2), ...
        transl2(2,3)*trot2(pi/2), 'absTol', 1e-6);
end

function rot2_test(testCase)
    verifyEqual(testCase,  rot2(0), ...
        [
        1     0
        0     1], 'absTol', 1e-6);
    verifyEqual(testCase,  trot2(0), ...
        eye(3,3), 'absTol', 1e-6);
    verifyEqual(testCase,  rot2(0), ...
        eye(2,2), 'absTol', 1e-6);
    verifyEqual(testCase,  trot2(pi/2), ...
        [
        0    -1     0
        1     0     0
        0     0     1], 'absTol', 1e-6);
    verifyEqual(testCase,  trot2(pi)*trot2(-pi/2), ...
        trot2(pi/2), 'absTol', 1e-6);
    verifyEqual(testCase,  rot2(pi)*rot2(-pi/2), ...
        rot2(pi/2), 'absTol', 1e-6);
end

%% angle/vector form
%    angvec2r                   - angle/vector to RM
function angvec2r_test(testCase)
    verifyEqual(testCase, angvec2r(0, [1 0 0]),...
        eye(3,3),'absTol',1e-4);

    verifyEqual(testCase, angvec2r( pi/2, [1 0 0]),...
        rotx(pi/2),'absTol',1e-4);
    
    verifyEqual(testCase, angvec2r( pi/2, [0 1 0]),...
        roty(pi/2),'absTol',1e-4);
    
    verifyEqual(testCase, angvec2r( pi/2, [0 0 1]),...
        rotz(pi/2),'absTol',1e-4);
 
    verifyError(testCase, @()angvec2r(1, [0 0 0]),'RTB:angvec2r:badarg');

    verifyError(testCase, @()angvec2r([1,2,3],0.1),'RTB:angvec2r:badarg');
    verifyError(testCase, @()angvec2r(1),'RTB:angvec2r:badarg');
end

%    angvec2tr                  - angle/vector to HT
function angvec2tr_test(testCase)
    
    verifyEqual(testCase, angvec2tr(0, [1 0 0]),...
        eye(4,4),'absTol',1e-4);
    
    verifyEqual(testCase, angvec2tr( pi/2, [1 0 0]),...
        trotx(pi/2),'absTol',1e-4);
    
    verifyEqual(testCase, angvec2tr( pi/2, [0 1 0]),...
        troty(pi/2),'absTol',1e-4);
    
    verifyEqual(testCase, angvec2tr( pi/2, [0 0 1]),...
        trotz(pi/2),'absTol',1e-4);
    
    verifyError(testCase, @()angvec2tr(1, [0 0 0]),'RTB:angvec2r:badarg');
    
    verifyError(testCase, @()angvec2tr([1,2,3],0.1),'RTB:angvec2r:badarg');
    verifyError(testCase, @()angvec2tr(1),'RTB:angvec2tr:badarg');
end
     
%    tr2angvec                  - HT/RM to angle/vector form
function tr2angvec_test(testCase)
	% null rotation
    [theta, v] = tr2angvec(eye(3,3));
    verifyEqual(testCase, theta, 0.0, 'absTol',1e-6);
    verifyEqual(testCase, v, [1 0 0], 'absTol',1e-6);
    
    % canonic rotations
    [theta, v] = tr2angvec(rotx(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [1 0 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(roty(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [0 1 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(rotz(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [0 0 1], 'absTol',1e-6);
    
    % null rotation
    [theta, v] = tr2angvec(eye(4,4));
    verifyEqual(testCase, theta, 0.0, 'absTol',1e-6);
    verifyEqual(testCase, v, [1 0 0], 'absTol',1e-6);
    
    % canonic rotations
    [theta, v] = tr2angvec(trotx(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [1 0 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(troty(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [0 1 0], 'absTol',1e-6);
    
    [theta, v] = tr2angvec(trotz(pi/2));
    verifyEqual(testCase, theta, pi/2, 'absTol',1e-6);
    verifyEqual(testCase, v, [0 0 1], 'absTol',1e-6);
     
    [theta, v] = tr2angvec(roty(pi/2), 'deg');
    verifyEqual(testCase, theta, 90, 'absTol',1e-6);
    verifyEqual(testCase, v, [0 1 0], 'absTol',1e-6);
    
    R = cat(3, rotx(pi/2), roty(pi/2), rotz(pi/2));
    [theta, v] = tr2angvec(R);
    verifyEqual(testCase, theta, pi/2*[1 1 1]', 'absTol',1e-6);
    verifyEqual(testCase, v, eye(3,3), 'absTol',1e-6);
    
    T = cat(3, trotx(pi/2), troty(pi/2), trotz(pi/2));
    [theta, v] = tr2angvec(T);
    verifyEqual(testCase, theta, pi/2*[1 1 1]', 'absTol',1e-6);
    verifyEqual(testCase, v, eye(3,3), 'absTol',1e-6);
    
    %test for scalar input
    verifyError(testCase, @()tr2angvec(1), 'RTB:tr2angvec:badarg');
end
    


%% 3-angle forms

function eul2r_test(testCase)
    
    % ZYZ
    r2d = 180/pi;
    
    R = rotz(0.1) * roty(0.2) * rotz(0.3);
    
    verifyEqual(testCase, eul2r(0.1, 0.2, 0.3), R, 'absTol',1e-4); 
    verifyEqual(testCase, eul2r([0.1, 0.2, 0.3]), R, 'absTol',1e-4);
    verifyEqual(testCase, eul2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), R, 'absTol',1e-4); 
    verifyEqual(testCase, eul2r([0.1, 0.2, 0.3]*r2d, 'deg'), R, 'absTol',1e-4);
    
    % trajectory case
    Rs = eul2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);
    
    Rs = eul2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);

    %test for scalar input
    verifyError(testCase, @()eul2r(1),'RTB:eul2r:badarg');
end
    
%    eul2tr                     - Euler angles to HT
function eul2tr_test(testCase)
    r2d = 180/pi;
    
    T = trotz(0.1) * troty(0.2) * trotz(0.3);
    
    verifyEqual(testCase, eul2tr(0.1, 0.2, 0.3), T, 'absTol',1e-4); 
    verifyEqual(testCase, eul2tr([0.1, 0.2, 0.3]), T, 'absTol',1e-4);
    verifyEqual(testCase, eul2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), T, 'absTol',1e-4); 
    verifyEqual(testCase, eul2tr([0.1, 0.2, 0.3]*r2d, 'deg'), T, 'absTol',1e-4);
    
    % trajectory case
    Ts = eul2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);
    
    Ts = eul2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);

    %test for scalar input
    verifyError(testCase, @()eul2tr(1),'RTB:eul2r:badarg');
end

function rpy2r_test(testCase)
    
    r2d = 180/pi;
    
    R = rotz(0.1) * roty(0.2) * rotx(0.3);
    
    verifyEqual(testCase, rpy2r(0.1, 0.2, 0.3), R, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2r([0.1, 0.2, 0.3]), R, 'absTol',1e-4);
    verifyEqual(testCase, rpy2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), R, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2r([0.1, 0.2, 0.3]*r2d, 'deg'), R, 'absTol',1e-4);
    
    % trajectory case
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);
    
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);

    R = rotx(0.1) * roty(0.2) * rotz(0.3);
    
    verifyEqual(testCase, rpy2r(0.1, 0.2, 0.3, 'xyz'), R, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2r([0.1, 0.2, 0.3], 'xyz'), R, 'absTol',1e-4);
    verifyEqual(testCase, rpy2r(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', 'xyz'), R, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2r([0.1, 0.2, 0.3]*r2d, 'deg', 'xyz'), R, 'absTol',1e-4);  
    
    % trajectory case
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3], 'xyz');
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);
    
    Rs = rpy2r( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'xyz', 'deg');
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), R, 'absTol',1e-4);
    
    %test for scalar input
    verifyError(testCase, @()rpy2tr(1),'RTB:rpy2r:badarg');
end

function rpy2tr_test(testCase)

    r2d = 180/pi;
    
    T = trotz(0.1) * troty(0.2) * trotx(0.3);
    
    verifyEqual(testCase, rpy2tr(0.1, 0.2, 0.3), T, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2tr([0.1, 0.2, 0.3]), T, 'absTol',1e-4);
    verifyEqual(testCase, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg'), T, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg'), T, 'absTol',1e-4);
    
    % trajectory case
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]);
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);
    
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'deg');
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);

    T = trotx(0.1) * troty(0.2) * trotz(0.3);
    
    verifyEqual(testCase, rpy2tr(0.1, 0.2, 0.3, 'xyz'), T, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2tr([0.1, 0.2, 0.3], 'xyz'), T, 'absTol',1e-4);
    verifyEqual(testCase, rpy2tr(0.1*r2d, 0.2*r2d, 0.3*r2d, 'deg', 'xyz'), T, 'absTol',1e-4); 
    verifyEqual(testCase, rpy2tr([0.1, 0.2, 0.3]*r2d, 'deg', 'xyz'), T, 'absTol',1e-4);  
    
    % trajectory case
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3], 'xyz');
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);
    
    Ts = rpy2tr( [0.1, 0.2, 0.3; 0.1 0.2 0.3; 0.1 0.2 0.3]*r2d, 'xyz', 'deg');
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), T, 'absTol',1e-4);
    
    %test for scalar input
    verifyError(testCase, @()rpy2tr(1),'RTB:rpy2r:badarg');
end

function tr2eul_test(testCase)

    eul = [0.1 0.2 0.3];
    R = eul2r(eul);
    verifyEqual(testCase, tr2eul(R), eul,'absTol',1e-4);
    verifyEqual(testCase, tr2eul(R, 'deg'), eul*180/pi,'absTol',1e-4);

    Rs = cat(3, R, R, R, R);
    x = tr2eul(Rs);
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), eul,'absTol',1e-4);
    x = tr2eul(Rs, 'deg');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), eul*180/pi,'absTol',1e-4);

    T = eul2tr(eul);
    verifyEqual(testCase, tr2eul(T), eul,'absTol',1e-4);
    verifyEqual(testCase, tr2eul(T, 'deg'), eul*180/pi,'absTol',1e-4);

    Ts = cat(3, T, T, T, T);
    x = tr2eul(Ts);
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), eul,'absTol',1e-4);
    x = tr2eul(Ts, 'deg');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), eul*180/pi,'absTol',1e-4);

    %test for scalar input
    verifyError(testCase, @()tr2eul(1),'MATLAB:badsubscript');
end
    
function tr2rpy_test(testCase)
    rpy = [0.1 0.2 0.3];
    R = rpy2r(rpy);
    verifyEqual(testCase, tr2rpy(R), rpy,'absTol',1e-4);
    verifyEqual(testCase, tr2rpy(R, 'deg'), rpy*180/pi,'absTol',1e-4);

    Rs = cat(3, R, R, R, R);
    x = tr2rpy(Rs);
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy,'absTol',1e-4);
    x = tr2rpy(Rs, 'deg');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy*180/pi,'absTol',1e-4);

    T = rpy2tr(rpy);
    verifyEqual(testCase, tr2rpy(T), rpy,'absTol',1e-4);
    verifyEqual(testCase, tr2rpy(T, 'deg'), rpy*180/pi,'absTol',1e-4);

    Ts = cat(3, T, T, T, T);
    x = tr2rpy(Ts);
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy,'absTol',1e-4);
    x = tr2rpy(Ts, 'deg');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy*180/pi,'absTol',1e-4);

    % xyz order
    R = rpy2r(rpy, 'xyz');
    verifyEqual(testCase, tr2rpy(R, 'xyz'), rpy,'absTol',1e-4);
    verifyEqual(testCase, tr2rpy(R, 'deg', 'xyz'), rpy*180/pi,'absTol',1e-4);

    Rs = cat(3, R, R, R, R);
    x = tr2rpy(Rs, 'xyz');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy,'absTol',1e-4);
    x = tr2rpy(Rs, 'deg', 'xyz');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy*180/pi,'absTol',1e-4);

    T = rpy2tr(rpy, 'xyz');
    verifyEqual(testCase, tr2rpy(T, 'xyz'), rpy,'absTol',1e-4);
    verifyEqual(testCase, tr2rpy(T, 'deg', 'xyz'), rpy*180/pi,'absTol',1e-4);

    Ts = cat(3, T, T, T, T);
    x = tr2rpy(Ts, 'xyz');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy,'absTol',1e-4);
    x = tr2rpy(Ts, 'deg', 'xyz');
    verifySize(testCase, x, [4 3]);
    verifyEqual(testCase, x(2,:), rpy*180/pi,'absTol',1e-4);

    %test for scalar input
    verifyError(testCase, @()tr2rpy(1),'MATLAB:badsubscript');
end
    
        

%    oa2r                       - orientation and approach vector to RM
function oa2r_test(testCase)
    %Unit test for oa2r with variables ([0 1 0] & [0 0 1]) 
    verifyEqual(testCase, oa2r([0 1 0], [0 0 1]),...
        [1     0     0
         0     1     0
         0     0     1],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()oa2r(1),'RTB:oa2r:badarg');
end

%    oa2tr                      - orientation and approach vector to HT
function oa2tr_test(testCase)
    %Unit test for oa2tr with variables ([0 1 0] & [0 0 1]) 
    verifyEqual(testCase, oa2tr([0 1 0], [0 0 1]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()oa2tr(1),'RTB:oa2tr:badarg');
end
    
%    r2t                        - RM to HT
function r2t_test(testCase)
    %Unit test for r2t

    % SO(3) case
    R = [1 2 3;4 5 6; 7 8 9];
    verifyEqual(testCase, r2t(R),...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absTol',1e-4);

    % sequence case
    Rs = cat(3, R, R, R);
    Ts = r2t(Rs);
    verifySize(testCase, Ts, [4 4 3]);
    verifyEqual(testCase, Ts(:,:,2), ...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absTol',1e-4);

    % SO(2) case
    R = [1 2; 3 4];
    verifyEqual(testCase, r2t(R),...
        [1 2 0; 3 4 0; 0 0 1],'absTol',1e-4);
end
   
%    t2r                        - HT to RM
function t2r_test(testCase)
    %Unit test for r2t with variables eul2tr([.1, .2, .3])

    % SO(3) case
    T = [1 2 3 4; 5 6 7 8; 9 10 11 12; 0 0 0 1];
    verifyEqual(testCase, t2r(T),...
        [1 2 3; 5 6 7; 9 10 11],'absTol',1e-4);

    % sequence case
    Ts = cat(3, T, T, T);
    Rs = t2r(Ts);
    verifySize(testCase, Rs, [3 3 3]);
    verifyEqual(testCase, Rs(:,:,2), ...
        [1 2 3; 5 6 7; 9 10 11],'absTol',1e-4);

    % SO(2) case
    T = [1 2 3; 4 5 6; 0 0 1];
    verifyEqual(testCase, t2r(T),...
        [1 2; 4 5],'absTol',1e-4);
end


 
     
%    trnorm                     - normalize HT
function trnorm_test(testCase)
        
    R = [0.9 0 0; .2 .6 .3; .1 .2 .4]';
    verifyEqual(testCase, det(trnorm(R)), 1, 'absTol', 1e-14);

    t = [1 2 3]';
    T = rt2tr(R, t);
    Tn = trnorm(T);
    verifyEqual(testCase, det(trnorm(t2r(Tn))), 1, 'absTol', 1e-14);
    verifyEqual(testCase, Tn(1:3,4), t);

    %test for scalar input
    verifyError(testCase, @()trnorm(1),'RTB:trnorm:badarg');    
end
          
     

function trlog_test(testCase)
    %unit tests for matrix expon stuff

    %%% SO(3) tests
    % zero rotation case
    verifyEqual(testCase, trlog( eye(3,3) ), skew([0 0 0]), 'absTol', 1e-6);

    % rotation by pi case
    verifyEqual(testCase, trlog( rotx(pi) ), skew([pi 0 0]), 'absTol', 1e-6);
    verifyEqual(testCase, trlog( roty(pi) ), skew([0 pi 0]), 'absTol', 1e-6);
    verifyEqual(testCase, trlog( rotz(pi) ), skew([0 0 pi]), 'absTol', 1e-6);

    % general case
    verifyEqual(testCase, trlog( rotx(0.2) ), skew([0.2 0 0]), 'absTol', 1e-6);
    verifyEqual(testCase, trlog( roty(0.3) ), skew([0 0.3 0]), 'absTol', 1e-6);
    verifyEqual(testCase, trlog( rotz(0.4) ), skew([0 0 0.4]), 'absTol', 1e-6);

    %%% SE(3) tests

    % pure translation
    verifyEqual(testCase, trlog( transl([1 2 3]) ), ...
        [0 0 0 1; 0 0 0 2; 0 0 0 3; 0 0 0 0], 'absTol', 1e-6);

    % mixture
    T = transl([1 2 3])*trotx(0.3);
    verifyEqual(testCase, trlog(T), logm(T), 'absTol', 1e-6);
    
    T = transl([1 2 3])*troty(0.3);
    verifyEqual(testCase, trlog(T), logm(T), 'absTol', 1e-6);
    
    verifyError(testCase, @()trlog(0),'RTB:trlog:badarg');

end

function trexp_test(testCase)
    %unit tests for matrix log stuff

    %%% SO(3) tests
    % zero rotation case
    verifyEqual(testCase, trexp(skew([0 0 0])), eye(3,3), 'absTol', 1e-6);

    % rotation by pi case
    verifyEqual(testCase, trexp(skew([pi 0 0])), rotx(pi), 'absTol', 1e-6);
    verifyEqual(testCase, trexp(skew([0 pi 0])), roty(pi), 'absTol', 1e-6);
    verifyEqual(testCase, trexp(skew([0 0 pi])), rotz(pi), 'absTol', 1e-6);

    % general case
    verifyEqual(testCase, trexp(skew([0.2 0 0])), rotx(0.2), 'absTol', 1e-6);
    verifyEqual(testCase, trexp(skew([0 0.3 0])), roty(0.3), 'absTol', 1e-6);
    verifyEqual(testCase, trexp(skew([0 0 0.4])), rotz(0.4), 'absTol', 1e-6);

    %%% SE(3) tests

    % pure translation
    verifyEqual(testCase, trexp([0 0 0 1; 0 0 0 2; 0 0 0 3; 0 0 0 0]), ...
        transl([1 2 3]), 'absTol', 1e-6);

    % mixture
    T = transl([1 2 3])*trotx(0.3);
    verifyEqual(testCase, trexp(logm(T)), T, 'absTol', 1e-6);
    
    T = transl([1 2 3])*troty(0.3);
    verifyEqual(testCase, trexp(logm(T)), T, 'absTol', 1e-6);
end

function twist_test(testCase)
    %2D twists
    
    % check basics work
    s = [1 2 3];
    tw = Twist(s);
    verifyEqual(testCase, tw.s, s, 'absTol', 1e-6);
    verifyEqual(testCase, tw.v', s(1:2), 'absTol', 1e-6);
    verifyEqual(testCase, tw.w, s(3), 'absTol', 1e-6);
    verifyEqual(testCase, tw.S, [skew(s(3)) [s(1:2)]'; 0 0 0], 'absTol', 1e-6);
    
    % check rendering
    c = tw.char;
    a = [tw tw tw];
    c = tw.char;
    
    % check overloaded +
    s2 = [4 6 5];
    tw2 = Twist(s2);
    sum = tw+tw2;
    verifyEqual(testCase, sum.s, tw.s+tw2.s, 'absTol', 1e-6);
    
    % check rotational twist
    tw = Twist('R', [1 2]);
    verifyEqual(testCase, tw.s, [2 -1 1], 'absTol', 1e-6);
    
    % check prismatic twist
    tw = Twist('P', [2 3]);
    verifyEqual(testCase, tw.s, [unit([2 3]) 0], 'absTol', 1e-6);
    tw = Twist('T', [2 3]);
    verifyEqual(testCase, tw.s, [unit([2 3]) 0], 'absTol', 1e-6);
    
    % check twist from SE(2)
    tw = Twist( trot2(0) );
    verifyEqual(testCase, tw.s, [0 0 0], 'absTol', 1e-6);
    tw = Twist( trot2(pi/2) );
    verifyEqual(testCase, tw.s, [0 0 pi/2], 'absTol', 1e-6);    
    tw = Twist( SE2(1,2,0) );
    verifyEqual(testCase, tw.s, [1 2 0], 'absTol', 1e-6);
    tw = Twist( SE2(1,2,pi/2) );
    verifyEqual(testCase, tw.s, [3*pi/4 pi/4 pi/2], 'absTol', 1e-6);

    % test expm and T
    verifyEqual(testCase, tw.T, expm(tw.S), 'absTol', 1e-6);
    verifyEqual(testCase, tw.expm, expm(tw.S), 'absTol', 1e-6);

    tw = Twist('R', [1 2]);
    verifyEqual(testCase, tw.T(pi/2), [0 -1 3; 1 0 1; 0 0 1], 'absTol', 1e-6);

    % 3D twists
    
    % check basics work
    s = [1 2 3 4 5 6];
    tw = Twist(s);
    verifyEqual(testCase, tw.s, s, 'absTol', 1e-6);
    verifyEqual(testCase, tw.v', s(1:3), 'absTol', 1e-6);
    verifyEqual(testCase, tw.w', s(4:6), 'absTol', 1e-6);
    verifyEqual(testCase, tw.S, [skew(s(4:6)) [s(1:3)]'; 0 0 0 0], 'absTol', 1e-6);
    
    % check rendering
    c = tw.char;
    a = [tw tw tw];
    c = tw.char;
    
    % check overloaded +
    s2 = [4 6 5 7 9 8];
    tw2 = Twist(s2);
    sum = tw+tw2;
    verifyEqual(testCase, sum.s, tw.s+tw2.s, 'absTol', 1e-6);
    
    % check rotational twist
    tw = Twist('R', [1 2 3], [0 0 0]);
    verifyEqual(testCase, tw.s, [0 0 0 unit([1 2 3])], 'absTol', 1e-6);
    
    % check prismatic twist
    tw = Twist('P', [1 2 3]);
    verifyEqual(testCase, tw.s, [unit([1 2 3]) 0 0 0 ], 'absTol', 1e-6);
    tw2 = Twist('T', [1 2 3]);
    verifyEqual(testCase, tw.s, tw2.s, 'absTol', 1e-6);
    
    % check twist from SE(3)
    tw = Twist( trotx(0) );
    verifyEqual(testCase, tw.s, [0 0 0  0 0 0], 'absTol', 1e-6);
    tw = Twist( trotx(pi/2) );
    verifyEqual(testCase, tw.s, [0 0 0  pi/2 0 0], 'absTol', 1e-6);    
    tw = Twist( troty(pi/2) );
    verifyEqual(testCase, tw.s, [0 0 0  0 pi/2 0], 'absTol', 1e-6);
    tw = Twist( trotz(pi/2) );
    verifyEqual(testCase, tw.s, [0 0 0  0 0 pi/2], 'absTol', 1e-6);
    
    tw = Twist( transl([1 2 3]) );
    verifyEqual(testCase, tw.s, [1 2 3  0 0 0], 'absTol', 1e-6);
    tw = Twist( transl([1 2 3])*troty(pi/2) );
    verifyEqual(testCase, tw.s, [-pi/2 2 pi  0 pi/2 0], 'absTol', 1e-6);

    % test expm and T
    verifyEqual(testCase, tw.T, expm(tw.S), 'absTol', 1e-6);
    verifyEqual(testCase, tw.expm, expm(tw.S), 'absTol', 1e-6);
    
    tw = Twist('R', [1 0 0], [0 0 0]);
    verifyEqual(testCase, tw.T(pi/2), trotx(pi/2), 'absTol', 1e-6);
    tw = Twist('R', [0 1 0], [0 0 0]);
    verifyEqual(testCase, tw.T(pi/2), troty(pi/2), 'absTol', 1e-6);
    tw = Twist('R', [0 0 1], [0 0 0]);
    verifyEqual(testCase, tw.T(pi/2), trotz(pi/2), 'absTol', 1e-6);
end

