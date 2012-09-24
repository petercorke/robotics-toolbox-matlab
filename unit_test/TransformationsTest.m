%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox

function test_suite = TestRobotToolboxHomogeneousTransformation
  initTestSuite;

%% Homogeneous transformations
%    angvec2r                   - angle/vector to RM
function angvec2r_test
    %Unit test for angvec2r with variables 0.1, [1,2,3]) 
    assertElementsAlmostEqual(angvec2r( 0.1, [1,2,3]),...
    [1.0000   -0.2895    0.2147
    0.3095    1.0150   -0.0699
   -0.1847    0.1298    1.0400],'absolute',1e-4);
    %Unit test for angvec2r with variables 0, [0 0 0]) 
    assertElementsAlmostEqual(angvec2r( 0, [0 0 0]),...
    [1     0     0
     0     1     0
     0     0     1],'absolute',1e-4);
    %Unit test for angvec2r with variables ([1,2,3],0.1) 
    assertExceptionThrown(@()angvec2r([1,2,3],0.1),'RTB:angvec2r:badarg');
    %Unit test for angvec2r with variables (1)
    assertExceptionThrown(@()angvec2r(1),'RTB:angvec2r:badarg');


%    angvec2tr                  - angle/vector to HT
function angvec2tr_test
    %Unit test for angvec2r with variables 0.1, [1,2,3]) 
    assertElementsAlmostEqual(angvec2tr( 0.1, [1,2,3]),...
    [1.0000   -0.2895    0.2147         0
    0.3095    1.0150   -0.0699         0
   -0.1847    0.1298    1.0400         0
         0         0         0    1.0000],'absolute',1e-4);
    %Unit test for angvec2tr with variables 0, [0 0 0]) 
    assertElementsAlmostEqual(angvec2tr( 0, [0 0 0]),...
    [1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1],'absolute',1e-4);
    %Unit test for angvec2tr with variables ([1,2,3],0.1) 
    assertExceptionThrown(@()angvec2tr([1,2,3],0.1),'RTB:angvec2r:badarg');
    %Unit test for angvec2tr with variables (1)
    assertExceptionThrown(@()angvec2tr(1),'RTB:angvec2tr:badarg');
     
%    eul2r                      - Euler angles to RM
function eul2r_test
    assertElementsAlmostEqual(eul2r(0, 0, 0),...
        [1     0     0
         0     1     0
         0     0     1], ...
    'absolute',1e-4);

    assertElementsAlmostEqual(eul2r(.1, .2, .3),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(eul2r(.1, .2, .3, 'deg'),...
        [1.0000   -0.0070    0.0035
        0.0070    1.0000    0.0000
       -0.0035    0.0000    1.0000], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(eul2r([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(eul2r([.1, .2, .3], 'deg'),...
        [1.0000   -0.0070    0.0035
        0.0070    1.0000    0.0000
       -0.0035    0.0000    1.0000], ...
        'absolute',1e-4);

    % trajectory case
    R = eul2r( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    assertIsSize(R, [3 3 3]);

    assertElementsAlmostEqual(R(:,:,2), ...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801], ...
         'absolute',1e-4);

    %test for scalar input
    assertExceptionThrown(@()eul2r(1),'');
    
%    eul2tr                     - Euler angles to HT
function eul2tr_test
    %Unit test for eul2tr with variables (0, 0, 0)
    assertElementsAlmostEqual(eul2tr(0, 0, 0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);

    assertElementsAlmostEqual(eul2tr(.1, .2, .3),...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual( eul2tr(.1, .2, .3, 'deg'), ...
        [1.0000   -0.0070    0.0035         0
        0.0070    1.0000    0.0000         0
       -0.0035    0.0000    1.0000         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual(eul2tr([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual(eul2tr([.1, .2, .3], 'deg'),...
        [1.0000   -0.0070    0.0035         0
        0.0070    1.0000    0.0000         0
       -0.0035    0.0000    1.0000         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    % trajectory case
    T = eul2tr( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    assertIsSize(T, [4 4 3]);

    assertElementsAlmostEqual(T(:,:,2), ...
        [0.9021   -0.3836    0.1977         0
        0.3875    0.9216    0.0198         0
       -0.1898    0.0587    0.9801         0
         0         0         0    1.0000], ...
     'absolute', 1e-4);

    %test for scalar input
    assertExceptionThrown(@()eul2tr(1),'');
%    oa2r                       - orientation and approach vector to RM
function oa2r_test
    %Unit test for oa2r with variables ([0 1 0] & [0 0 1]) 
    assertElementsAlmostEqual(oa2r([0 1 0], [0 0 1]),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()oa2r(1),'RTB:oa2r:badarg');

%    oa2tr                      - orientation and approach vector to HT
function oa2tr_test
    %Unit test for oa2tr with variables ([0 1 0] & [0 0 1]) 
    assertElementsAlmostEqual(oa2tr([0 1 0], [0 0 1]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()oa2tr(1),'RTB:oa2tr:badarg');
    
%    r2t                        - RM to HT
function r2t_test
    %Unit test for r2t

    % SO(3) case
    R = [1 2 3;4 5 6; 7 8 9];
    assertElementsAlmostEqual(r2t(R),...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absolute',1e-4);

    % sequence case
    Rs = cat(3, R, R, R);
    Ts = r2t(Rs);
    assertIsSize(Ts, [4 4 3]);
    assertElementsAlmostEqual(Ts(:,:,2), ...
        [1 2 3 0; 4 5 6 0; 7 8 9 0; 0 0 0 1],'absolute',1e-4);

    % SO(2) case
    R = [1 2; 3 4];
    assertElementsAlmostEqual(r2t(R),...
        [1 2 0; 3 4 0; 0 0 1],'absolute',1e-4);

    
%    rotx                       - RM for rotation about X-axis
function rotx_test
    %Unit test for rotz
    assertElementsAlmostEqual(rotx(0.1),...
        [1.0000 0 0 ; 0 0.9950 -0.0998 ; 0 0.0998 0.9950 ],'absolute',1e-4);
    assertElementsAlmostEqual(rotx(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absolute',1e-4);
     %test for non-scalar input
    assertExceptionThrown(@()rotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
    
%    roty                       - RM for rotation about Y-axis
function roty_test
    %Unit test for roty
    assertElementsAlmostEqual(roty(0.1),...
        [0.9950 0 0.0998 ;0 1.0000 0 ;-0.0998 0 0.9950 ],'absolute',1e-4);
    assertElementsAlmostEqual(roty(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absolute',1e-4);
     %test for non-scalar input
    assertExceptionThrown(@()roty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
    
%    rotz                       - RM for rotation about Z-axis
function rotz_test
    %Unit test for rotz
    assertElementsAlmostEqual(rotz(0.1),...
        [0.9950 -0.0998 0 ; 0.0998 0.9950 0 ;0 0 1.0000],'absolute',1e-4);
    assertElementsAlmostEqual(rotz(0),...
        [1     0     0
         0     1     0
         0     0     1 ],'absolute',1e-4);
     %test for non-scalar input
    assertExceptionThrown(@()rotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');
    
%    rpy2r                      - roll/pitch/yaw angles to RM
function rpy2r_test
    %Unit test for rpy2r with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(rpy2r(.1, .2, .3),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
        'absolute',1e-4);
    assertElementsAlmostEqual(rpy2r(.1, .2, .3, 'deg'),...
        [1.0000   -0.0052    0.0035
        0.0052    1.0000   -0.0017
       -0.0035    0.0018    1.0000], ...
        'absolute',1e-4);
    assertElementsAlmostEqual(rpy2r(.1, .2, .3, 'zyx'),...
        [0.9752   -0.0370    0.2184
        0.0978    0.9564   -0.2751
       -0.1987    0.2896    0.9363], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(rpy2r([.1, .2, .3]),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(rpy2r([.1, .2, .3], 'deg'),...
        [1.0000   -0.0052    0.0035
        0.0052    1.0000   -0.0017
       -0.0035    0.0018    1.0000], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(rpy2r([.1, .2, .3], 'zyx'),...
        [0.9752   -0.0370    0.2184
        0.0978    0.9564   -0.2751
       -0.1987    0.2896    0.9363], ...
        'absolute',1e-4);

    assertElementsAlmostEqual(rpy2r([0 0 0]),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);

    % trajectory case
    R = rpy2r( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    assertIsSize(R, [3 3 3]);

    assertElementsAlmostEqual(R(:,:,2), ...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752], ...
         'absolute',1e-4);

        
%    rpy2tr                     - roll/pitch/yaw angles to HT
function rpy2tr_test
    %Unit test for rpy2tr with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(rpy2tr(.1, .2, .3),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4);

    assertElementsAlmostEqual( rpy2tr(.1, .2, .3, 'deg'), ...
        [1.0000   -0.0052    0.0035         0
        0.0052    1.0000   -0.0017         0
       -0.0035    0.0018    1.0000         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual( rpy2tr(.1, .2, .3, 'xyz'), ...
        [0.9363   -0.2896    0.1987         0
        0.3130    0.9447   -0.0978         0
       -0.1593    0.1538    0.9752         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual(rpy2tr([.1, .2, .3]),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4);

    assertElementsAlmostEqual(rpy2tr([.1, .2, .3], 'deg'),...
        [1.0000   -0.0052    0.0035         0
        0.0052    1.0000   -0.0017         0
       -0.0035    0.0018    1.0000         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    assertElementsAlmostEqual(rpy2tr([.1, .2, .3], 'xyz'),...
        [0.9363   -0.2896    0.1987         0
        0.3130    0.9447   -0.0978         0
       -0.1593    0.1538    0.9752         0
             0         0         0    1.0000], ...
     'absolute', 1e-4);

    % trajectory case
    T = rpy2tr( [.1, .2, .3; .1 .2 .3; .1 .2 .3]);
    assertIsSize(T, [4 4 3]);

    assertElementsAlmostEqual(T(:,:,2), ...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4);

    %Unit test for rpy2tr with variables ([0 0 0])
    assertElementsAlmostEqual(rpy2tr( [0 0 0]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()rpy2tr(1),'');
        
%    t2r                        - HT to RM
function t2r_test
    %Unit test for r2t with variables eul2tr([.1, .2, .3])

    % SO(3) case
    T = [1 2 3 4; 5 6 7 8; 9 10 11 12; 0 0 0 1];
    assertElementsAlmostEqual(t2r(T),...
        [1 2 3; 5 6 7; 9 10 11],'absolute',1e-4);

    % sequence case
    Ts = cat(3, T, T, T);
    Rs = t2r(Ts);
    assertIsSize(Rs, [3 3 3]);
    assertElementsAlmostEqual(Rs(:,:,2), ...
        [1 2 3; 5 6 7; 9 10 11],'absolute',1e-4);

    % SO(2) case
    T = [1 2 3; 4 5 6; 0 0 1];
    assertElementsAlmostEqual(t2r(T),...
        [1 2; 4 5],'absolute',1e-4);
    
%    tr2angvec                  - HT/RM to angle/vector form
% CHECK OUTPUT OF THIS FUNCTION!!!!!!!!!!!!!!!!!!!!!!
function tr2angvec_test
    % unit test for tr2angvec using a tr matrix 
    [theta, v] = tr2angvec(eye(4,4));
    assertElementsAlmostEqual(theta,...
        0.0,'absolute',1e-4);

    % unit test for tr2angvec using a tr matrix 
    [theta, v] = tr2angvec(eul2tr([.1, .2, .3]));
    assertElementsAlmostEqual(theta,...
        0.4466,'absolute',1e-4);
    assertElementsAlmostEqual(v,...
        [0.0450    0.4486    0.8926],'absolute',1e-4);
    %test with a r matrix
    [theta, v] = tr2angvec(eul2r([.3, .2, .1]));
    assertElementsAlmostEqual(theta,...
        0.4466,'absolute',1e-4);
    assertElementsAlmostEqual(v,...
        [-0.0450    0.4486    0.8926],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()tr2angvec(1), 'RTB:t2r:badarg');
    
%    tr2eul                     - HT/RM to Euler angles
function tr2eul_test
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2eul(eul2tr([.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absolute',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2eul(eul2tr([0 0 0])),...
        [0 0 0],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()tr2eul(1),'MATLAB:badsubscript');
    
%    tr2rpy                     - HT/RM to roll/pitch/yaw angles
function tr2rpy_test
    %Unit test for rpy2r with variables [.1, .2, .3]
     assertElementsAlmostEqual(tr2rpy(rpy2tr( [.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absolute',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2rpy(eul2tr([0 0 0])),...
        [0 0 0],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()tr2rpy(1),'MATLAB:badsubscript');
    
%    transl                     - set or extract the translational component of HT
function transl_test
    %Unit test for transl with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(transl(0.1, 0.2, 0.3),...
        [1.0000         0         0    0.1000
              0    1.0000         0    0.2000
              0         0    1.0000    0.3000
              0         0         0    1.0000],'absolute',1e-4);
    %Unit test for transl with variables ([0.1, 0.2, 0.3])
    assertElementsAlmostEqual(transl([0.1, 0.2, 0.3]),...
        [1.0000         0         0    0.1000
              0    1.0000         0    0.2000
              0         0    1.0000    0.3000
              0         0         0    1.0000],'absolute',1e-4);
    %Unit test for transl with variables [0 0 0]
    assertElementsAlmostEqual(transl([0 0 0] ),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    %Unit test for transl with variable (1)
    assertElementsAlmostEqual(transl(1),...
        [1     0     0     1
         0     1     0     1
         0     0     1     1
         0     0     0     1],'absolute',1e-4);
    
     
%    trnorm                     - normalize HT
function trnorm_test
    %Unit test for oa2r with variables tr matrix generated with oa2tr
    assertElementsAlmostEqual(trnorm(rpy2tr( [.1, .2, .3])),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4); 
    %test for scalar input
    assertExceptionThrown(@()trnorm(1),'RTB:trnorm:badarg');    
          
     
function rot_test
    assertElementsAlmostEqual(rotx(0), eye(3), 'absolute', 1e-9);
    assertElementsAlmostEqual(roty(0), eye(3), 'absolute', 1e-9);
    assertElementsAlmostEqual(rotz(0), eye(3), 'absolute', 1e-9);

    assertElementsAlmostEqual(rotx(pi/2), [1 0 0; 0 0 -1; 0 1 0], 'absolute', 1e-9);
    assertElementsAlmostEqual(roty(pi/2), [0 0 1; 0 1 0; -1 0 0], 'absolute', 1e-9);
    assertElementsAlmostEqual(rotz(pi/2), [0 -1 0; 1 0 0; 0 0 1], 'absolute', 1e-9);

    assertElementsAlmostEqual(trotx(0), eye(4), 'absolute', 1e-9);
    assertElementsAlmostEqual(troty(0), eye(4), 'absolute', 1e-9);
    assertElementsAlmostEqual(trotz(0), eye(4), 'absolute', 1e-9);

    assertElementsAlmostEqual(trotx(pi/2), [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1], 'absolute', 1e-9);
    assertElementsAlmostEqual(troty(pi/2), [0 0 1 0; 0 1 0 0; -1 0 0 0; 0 0 0 1], 'absolute', 1e-9);
    assertElementsAlmostEqual(trotz(pi/2), [0 -1 0 0; 1 0 0 0; 0 0 1 0; 0 0 0 1], 'absolute', 1e-9);

    assertElementsAlmostEqual(rotx(pi/2), rotx(90, 'deg'), 'absolute', 1e-9);
    assertElementsAlmostEqual(roty(pi/2), roty(90, 'deg'), 'absolute', 1e-9);
    assertElementsAlmostEqual(rotz(pi/2), rotz(90, 'deg'), 'absolute', 1e-9);

    assertElementsAlmostEqual(trotx(pi/2), trotx(90, 'deg'), 'absolute', 1e-9);
    assertElementsAlmostEqual(troty(pi/2), troty(90, 'deg'), 'absolute', 1e-9);
    assertElementsAlmostEqual(trotz(pi/2), trotz(90, 'deg'), 'absolute', 1e-9);

%    trotx                      - HT for rotation about X-axis
function trotx_test
    %Unit test for trotz
    assertElementsAlmostEqual(trotx(0.1),...
        [1.0000 0 0 0
         0 0.9950 -0.0998 0
         0 0.0998  0.9950 0
         0 0 0 1.0000],'absolute',1e-4);
    assertElementsAlmostEqual(trotx(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absolute',1e-4);
    %test for non-scalar input
    assertExceptionThrown(@()trotx([1 2 3]),'MATLAB:catenate:dimensionMismatch');
     
    
%    troty                      - HT for rotation about Y-axis
function troty_test
    %Unit test for troty
    assertElementsAlmostEqual(troty(0.1),...
        [0.9950 0 0.0998 0
        0 1.0000 0 0
        -0.0998 0 0.9950 0
        0 0 0 1.0000],'absolute',1e-4);
    assertElementsAlmostEqual(troty(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absolute',1e-4);
    %test for non-scalar input
    assertExceptionThrown(@()troty([1 2 3]),'MATLAB:catenate:dimensionMismatch');
    
%    trotz                      - HT for rotation about Z-axis
function trotz_test
    %Unit test for trotz
    assertElementsAlmostEqual(trotz(0.1),...
        [0.9950 -0.0998 0 0
        0.0998 0.9950 0 0
        0 0 1.0000 0
        0 0 0 1.0000],'absolute',1e-4);
    assertElementsAlmostEqual(trotz(0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1 ],'absolute',1e-4);
    %test for non-scalar input
    assertExceptionThrown(@()trotz([1 2 3]),'MATLAB:catenate:dimensionMismatch');


