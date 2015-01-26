%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox

function tests = TransformationsTest
  tests = functiontests(localfunctions);
end

%% Homogeneous transformations
%    angvec2r                   - angle/vector to RM
function angvec2r_test(testCase)
    %Unit test for angvec2r with variables 0.1, [1,2,3]) 
    verifyEqual(testCase, angvec2r( 0.1, [1,2,3]),...
    [1.0000   -0.2895    0.2147
    0.3095    1.0150   -0.0699
   -0.1847    0.1298    1.0400],'absTol',1e-4);
    %Unit test for angvec2r with variables 0, [0 0 0]) 
    verifyEqual(testCase, angvec2r( 0, [0 0 0]),...
    [1     0     0
     0     1     0
     0     0     1],'absTol',1e-4);
    %Unit test for angvec2r with variables ([1,2,3],0.1) 
    verifyError(testCase, @()angvec2r([1,2,3],0.1),'RTB:angvec2r:badarg');
    %Unit test for angvec2r with variables (1)
    verifyError(testCase, @()angvec2r(1),'RTB:angvec2r:badarg');
end

%    angvec2tr                  - angle/vector to HT
function angvec2tr_test(testCase)
    %Unit test for angvec2r with variables 0.1, [1,2,3]) 
    verifyEqual(testCase, angvec2tr( 0.1, [1,2,3]),...
    [1.0000   -0.2895    0.2147         0
    0.3095    1.0150   -0.0699         0
   -0.1847    0.1298    1.0400         0
         0         0         0    1.0000],'absTol',1e-4);
    %Unit test for angvec2tr with variables 0, [0 0 0]) 
    verifyEqual(testCase, angvec2tr( 0, [0 0 0]),...
    [1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1],'absTol',1e-4);
    %Unit test for angvec2tr with variables ([1,2,3],0.1) 
    verifyError(testCase, @()angvec2tr([1,2,3],0.1),'RTB:angvec2r:badarg');
    %Unit test for angvec2tr with variables (1)
    verifyError(testCase, @()angvec2tr(1),'RTB:angvec2tr:badarg');
end
     
%    eul2r                      - Euler angles to RM
function eul2r_test(testCase)
    verifyEqual(testCase, eul2r(0, 0, 0),...
        [1     0     0
         0     1     0
         0     0     1], ...
    'absTol',1e-4);

    verifyEqual(testCase, eul2r(.1, .2, .3),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
        'absTol',1e-4);

    verifyEqual(testCase, eul2r(.1, .2, .3, 'deg'),...
        [1.0000   -0.0070    0.0035
        0.0070    1.0000    0.0000
       -0.0035    0.0000    1.0000], ...
        'absTol',1e-4);

    verifyEqual(testCase, eul2r([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
        'absTol',1e-4);

    verifyEqual(testCase, eul2r([.1, .2, .3], 'deg'),...
        [1.0000   -0.0070    0.0035
        0.0070    1.0000    0.0000
       -0.0035    0.0000    1.0000], ...
        'absTol',1e-4);

    % trajectory case
    R = eul2r( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    verifySize(testCase, R, [3 3 3]);

    verifyEqual(testCase, R(:,:,2), ...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
         'absTol',1e-4);

    %test for scalar input
    verifyError(testCase, @()eul2r(1),'RTB:eul2r:badarg');
end
    
%    eul2tr                     - Euler angles to HT
function eul2tr_test(testCase)
    %Unit test for eul2tr with variables (0, 0, 0)
    verifyEqual(testCase, eul2tr(0, 0, 0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absTol',1e-4);

    verifyEqual(testCase, eul2tr(.1, .2, .3),...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase,  eul2tr(.1, .2, .3, 'deg'), ...
        [1.0000   -0.0070    0.0035         0
        0.0070    1.0000    0.0000         0
       -0.0035    0.0000    1.0000         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase, eul2tr([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase, eul2tr([.1, .2, .3], 'deg'),...
        [1.0000   -0.0070    0.0035         0
        0.0070    1.0000    0.0000         0
       -0.0035    0.0000    1.0000         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    % trajectory case
    T = eul2tr( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    verifySize(testCase, T, [4 4 3]);

    verifyEqual(testCase, T(:,:,2), ...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absTol', 1e-4);

    %test for scalar input
    verifyError(testCase, @()eul2tr(1),'RTB:eul2r:badarg');
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

    
%    rotx                       - RM for rotation about X-axis
function rotx_test(testCase)
    %Unit test for rotz
    verifyEqual(testCase, rotx(0.1),...
        [1.0000 0 0 ; 0 0.9950 -0.0998 ; 0 0.0998 0.9950 ],'absTol',1e-4);
    verifyEqual(testCase, rotx(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()rotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
%    roty                       - RM for rotation about Y-axis
function roty_test(testCase)
    %Unit test for roty
    verifyEqual(testCase, roty(0.1),...
        [0.9950 0 0.0998 ;0 1.0000 0 ;-0.0998 0 0.9950 ],'absTol',1e-4);
    verifyEqual(testCase, roty(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()roty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
%    rotz                       - RM for rotation about Z-axis
function rotz_test(testCase)
    %Unit test for rotz
    verifyEqual(testCase, rotz(0.1),...
        [0.9950 -0.0998 0 ; 0.0998 0.9950 0 ;0 0 1.0000],'absTol',1e-4);
    verifyEqual(testCase, rotz(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absTol',1e-4);
     %test for non-scalar input
    verifyError(testCase, @()rotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
%    rpy2r                      - roll/pitch/yaw angles to RM
function rpy2r_test(testCase)
    %Unit test for rpy2r with variables (0.1, 0.2, 0.3)
    verifyEqual(testCase, rpy2r(.1, .2, .3),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
        'absTol',1e-4);
    verifyEqual(testCase, rpy2r(.1, .2, .3, 'deg'),...
        [1.0000   -0.0052    0.0035
        0.0052    1.0000   -0.0017
       -0.0035    0.0018    1.0000], ...
        'absTol',1e-4);
    verifyEqual(testCase, rpy2r(.1, .2, .3, 'zyx'),...
        [0.9752   -0.0370    0.2184
        0.0978    0.9564   -0.2751
       -0.1987    0.2896    0.9363], ...
        'absTol',1e-4);

    verifyEqual(testCase, rpy2r([.1, .2, .3]),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
        'absTol',1e-4);

    verifyEqual(testCase, rpy2r([.1, .2, .3], 'deg'),...
        [1.0000   -0.0052    0.0035
        0.0052    1.0000   -0.0017
       -0.0035    0.0018    1.0000], ...
        'absTol',1e-4);

    verifyEqual(testCase, rpy2r([.1, .2, .3], 'zyx'),...
        [0.9752   -0.0370    0.2184
        0.0978    0.9564   -0.2751
       -0.1987    0.2896    0.9363], ...
        'absTol',1e-4);

    verifyEqual(testCase, rpy2r([0 0 0]),...
        [1     0     0
         0     1     0
         0     0     1],'absTol',1e-4);

    % trajectory case
    R = rpy2r( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    verifySize(testCase, R, [3 3 3]);

    verifyEqual(testCase, R(:,:,2), ...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
         'absTol',1e-4);
end

        
%    rpy2tr                     - roll/pitch/yaw angles to HT
function rpy2tr_test(testCase)
    %Unit test for rpy2tr with variables (0.1, 0.2, 0.3)
    verifyEqual(testCase, rpy2tr(.1, .2, .3),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absTol',1e-4);

    verifyEqual(testCase,  rpy2tr(.1, .2, .3, 'deg'), ...
        [1.0000   -0.0052    0.0035         0
        0.0052    1.0000   -0.0017         0
       -0.0035    0.0018    1.0000         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase,  rpy2tr(.1, .2, .3, 'xyz'), ...
        [0.9363   -0.2896    0.1987         0
        0.3130    0.9447   -0.0978         0
       -0.1593    0.1538    0.9752         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase, rpy2tr([.1, .2, .3]),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absTol',1e-4);

    verifyEqual(testCase, rpy2tr([.1, .2, .3], 'deg'),...
        [1.0000   -0.0052    0.0035         0
        0.0052    1.0000   -0.0017         0
       -0.0035    0.0018    1.0000         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    verifyEqual(testCase, rpy2tr([.1, .2, .3], 'xyz'),...
        [0.9363   -0.2896    0.1987         0
        0.3130    0.9447   -0.0978         0
       -0.1593    0.1538    0.9752         0
             0         0         0    1.0000], ...
     'absTol', 1e-4);

    % trajectory case
    T = rpy2tr( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    verifySize(testCase, T, [4 4 3]);

    verifyEqual(testCase, T(:,:,2), ...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absTol',1e-4);

    %Unit test for rpy2tr with variables ([0 0 0])
    verifyEqual(testCase, rpy2tr( [0 0 0]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()rpy2tr(1),'RTB:rpy2r:badarg');
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
    
%    tr2angvec                  - HT/RM to angle/vector form
% CHECK OUTPUT OF THIS FUNCTION!!!!!!!!!!!!!!!!!!!!!!
function tr2angvec_test(testCase)
    % unit test for tr2angvec using a tr matrix 
    [theta, v] = tr2angvec(eye(4,4));
    verifyEqual(testCase, theta,...
        0.0,'absTol',1e-4);

    % unit test for tr2angvec using a tr matrix 
    [theta, v] = tr2angvec(eul2tr([.1, .2, .3]));
    verifyEqual(testCase, theta,...
        0.4466,'absTol',1e-4);
    verifyEqual(testCase, v,...
        [0.0450    0.4486    0.8926],'absTol',1e-4);
    %test with a r matrix
    [theta, v] = tr2angvec(eul2r([.3, .2, .1]));
    verifyEqual(testCase, theta,...
        0.4466,'absTol',1e-4);
    verifyEqual(testCase, v,...
        [-0.0450    0.4486    0.8926],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()tr2angvec(1), 'RTB:t2r:badarg');
end
    
%    tr2eul                     - HT/RM to Euler angles
function tr2eul_test(testCase)
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    verifyEqual(testCase, tr2eul(eul2tr([.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absTol',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    verifyEqual(testCase, tr2eul(eul2tr([0 0 0])),...
        [0 0 0],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()tr2eul(1),'MATLAB:badsubscript');
end
    
%    tr2rpy                     - HT/RM to roll/pitch/yaw angles
function tr2rpy_test(testCase)
    %Unit test for rpy2r with variables [.1, .2, .3]
     verifyEqual(testCase, tr2rpy(rpy2tr( [.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absTol',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    verifyEqual(testCase, tr2rpy(eul2tr([0 0 0])),...
        [0 0 0],'absTol',1e-4);
    %test for scalar input
    verifyError(testCase, @()tr2rpy(1),'MATLAB:badsubscript');
end
    
%    transl                     - set or extract the translational component of HT
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
    
     
%    trnorm                     - normalize HT
function trnorm_test(testCase)
    %Unit test for oa2r with variables tr matrix generated with oa2tr
    verifyEqual(testCase, trnorm(rpy2tr( [.1, .2, .3])),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absTol',1e-4); 
    %test for scalar input
    verifyError(testCase, @()trnorm(1),'RTB:trnorm:badarg');    
end
          
     
function rot_test(testCase)
    verifyEqual(testCase, rotx(0), eye(3), 'absTol', 1e-9);
    verifyEqual(testCase, roty(0), eye(3), 'absTol', 1e-9);
    verifyEqual(testCase, rotz(0), eye(3), 'absTol', 1e-9);

    verifyEqual(testCase, rotx(pi/2), [1 0 0; 0 0 -1; 0 1 0], 'absTol', 1e-9);
    verifyEqual(testCase, roty(pi/2), [0 0 1; 0 1 0; -1 0 0], 'absTol', 1e-9);
    verifyEqual(testCase, rotz(pi/2), [0 -1 0; 1 0 0; 0 0 1], 'absTol', 1e-9);

    verifyEqual(testCase, trotx(0), eye(4), 'absTol', 1e-9);
    verifyEqual(testCase, troty(0), eye(4), 'absTol', 1e-9);
    verifyEqual(testCase, trotz(0), eye(4), 'absTol', 1e-9);

    verifyEqual(testCase, trotx(pi/2), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1], 'absTol', 1e-9);
    verifyEqual(testCase, troty(pi/2), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1], 'absTol', 1e-9);
    verifyEqual(testCase, trotz(pi/2), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1], 'absTol', 1e-9);

    verifyEqual(testCase, rotx(pi/2), rotx(90, 'deg'), 'absTol', 1e-9);
    verifyEqual(testCase, roty(pi/2), roty(90, 'deg'), 'absTol', 1e-9);
    verifyEqual(testCase, rotz(pi/2), rotz(90, 'deg'), 'absTol', 1e-9);

    verifyEqual(testCase, trotx(pi/2), trotx(90, 'deg'), 'absTol', 1e-9);
    verifyEqual(testCase, troty(pi/2), troty(90, 'deg'), 'absTol', 1e-9);
    verifyEqual(testCase, trotz(pi/2), trotz(90, 'deg'), 'absTol', 1e-9);
end

%    trotx                      - HT for rotation about X-axis
function trotx_test(testCase)
    %Unit test for trotz
    verifyEqual(testCase, trotx(0.1),...
        [1.0000 0 0 0
         0 0.9950 -0.0998 0
         0 0.0998  0.9950 0
         0 0 0 1.0000],'absTol',1e-4);
    verifyEqual(testCase, trotx(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absTol',1e-4);
    %test for non-scalar input
    verifyError(testCase, @()trotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
     
    
%    troty                      - HT for rotation about Y-axis
function troty_test(testCase)
    %Unit test for troty
    verifyEqual(testCase, troty(0.1),...
        [0.9950 0 0.0998 0
        0 1.0000 0 0
        -0.0998 0 0.9950 0
        0 0 0 1.0000],'absTol',1e-4);
    verifyEqual(testCase, troty(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absTol',1e-4);
    %test for non-scalar input
    verifyError(testCase, @()troty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end
    
%    trotz                      - HT for rotation about Z-axis
function trotz_test(testCase)
    %Unit test for trotz
    verifyEqual(testCase, trotz(0.1),...
        [0.9950 -0.0998 0 0
        0.0998 0.9950 0 0
        0 0 1.0000 0
        0 0 0 1.0000],'absTol',1e-4);
    verifyEqual(testCase, trotz(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absTol',1e-4);
    %test for non-scalar input
    verifyError(testCase, @()trotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');
end

function two_d_test(testCase)
    verifyEqual(testCase,  trot2(0.3), ...
            [    0.9553   -0.2955         0
                0.2955    0.9553         0
                         0         0    1.0000
            ], 'absTol',1e-4);
    verifyEqual(testCase,  rot2(0.3), ...
        [0.9553   -0.2955
            0.2955    0.9553
            ], 'absTol',1e-4);
    verifyEqual(testCase,  se2(1,2,0.3), ...
        [    0.9553   -0.2955    1.0000
            0.2955    0.9553    2.0000
             0         0    1.0000], 'absTol',1e-4);
    verifyEqual(testCase,  transl2(1,2), ...
         [     1     0     1
              0     1     2
               0     0     1 ], 'absTol',1e-4);
    verifyEqual(testCase,  transl2([1,2]), ...
         [     1     0     1
              0     1     2
               0     0     1 ], 'absTol',1e-4);
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
end
