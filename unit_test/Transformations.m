%% This is for testing the Homogeneous Transformation functions in the robotics Toolbox
function test_suite = TestRobotToolboxHomogeneousTransformation
  initTestSuite;
%% Homogeneous transformations
%    angvec2r                   - angle/vector to RM
function Test_angvec2r
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
    assertExceptionThrown(@()angvec2r([1,2,3],0.1),'MATLAB:badsubscript');
    %Unit test for angvec2r with variables (1)
    assertExceptionThrown(@()angvec2r(1),'MATLAB:inputArgUndefined');


%    angvec2tr                  - angle/vector to HT
function Test_angvec2tr
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
    assertExceptionThrown(@()angvec2tr([1,2,3],0.1),'MATLAB:badsubscript');
    %Unit test for angvec2tr with variables (1)
    assertExceptionThrown(@()angvec2tr(1),'MATLAB:inputArgUndefined');
     
%    eul2r                      - Euler angles to RM
function Test_eul2r
    %Unit test for eul2r with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(eul2r(.1, .2, .3),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801],'absolute',1e-4);
    %Unit test for eul2r with variables ([.1, .2, .3])
    assertElementsAlmostEqual(eul2r([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801],'absolute',1e-4);
    %Unit test for eul2r with variables ([.1, .2, .3; .4, .5, .6])
    expected_out(:,:,1)= [0.9021   -0.3836    0.1977
                          0.3875    0.9216    0.0198
                         -0.1898    0.0587    0.9801];
    expected_out(:,:,2)= [0.4472   -0.7778    0.4416
                          0.8021    0.5672    0.1867
                         -0.3957    0.2707    0.8776];
    assertElementsAlmostEqual(eul2r([.1, .2, .3; .4, .5, .6]),...
        expected_out,'absolute',1e-4);
    %Unit test for eul2r with variables (0, 0, 0)
    assertElementsAlmostEqual(eul2r(0, 0, 0),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()eul2r(1),'');
    
%    eul2tr                     - Euler angles to HT
function Test_eul2tr
    %Unit test for eul2tr with variables (.1, .2, .3)
    assertElementsAlmostEqual(eul2tr(.1, .2, .3),...
        [0.9021   -0.3836    0.1977         0
         0.3875    0.9216    0.0198         0
        -0.1898    0.0587    0.9801         0
              0         0         0    1.0000],'absolute',1e-4);
    %Unit test for eul2r with variables ([.1, .2, .3])
    assertElementsAlmostEqual(eul2tr([.1, .2, .3]),...
        [0.9021   -0.3836    0.1977         0
         0.3875    0.9216    0.0198         0
        -0.1898    0.0587    0.9801         0
              0         0         0    1.0000],'absolute',1e-4);   
    %Unit test for eul2tr with variables ([.1, .2, .3; .4, .5, .6])
    expected_out(:,:,1)= [0.9021   -0.3836    0.1977         0
                          0.3875    0.9216    0.0198         0
                         -0.1898    0.0587    0.9801         0
                               0         0         0    1.0000];
    expected_out(:,:,2)= [0.4472   -0.7778    0.4416         0
                          0.8021    0.5672    0.1867         0
                         -0.3957    0.2707    0.8776         0
                               0         0         0    1.0000];
    assertElementsAlmostEqual(eul2tr([.1, .2, .3; .4, .5, .6]),...
        expected_out,'absolute',1e-4) 
    %Unit test for eul2tr with variables (0, 0, 0)
    assertElementsAlmostEqual(eul2tr(0, 0, 0),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()eul2tr(1),'');
%    oa2r                       - orientation and approach vector to RM
function Test_oa2r
    %Unit test for oa2r with variables ([0 1 0] & [0 0 1]) 
    assertElementsAlmostEqual(oa2r([0 1 0], [0 0 1]),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()oa2r(1),'MATLAB:inputArgUndefined');

%    oa2tr                      - orientation and approach vector to HT
function Test_oa2tr
    %Unit test for oa2tr with variables ([0 1 0] & [0 0 1]) 
    assertElementsAlmostEqual(oa2tr([0 1 0], [0 0 1]),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()oa2tr(1),'MATLAB:inputArgUndefined');
    
%    r2t                        - RM to HT
function Test_r2t
    %Unit test for r2t
    assertElementsAlmostEqual(r2t(rotz(0.1)),...
        [0.9950 -0.0998 0 0; 0.0998 0.9950 0 0;0 0 1.0000 0;0 0 0 1.0000],'absolute',1e-4);
    assertElementsAlmostEqual(r2t(rotz(0)),...
        [1     0     0     0
         0     1     0     0
         0     0     1     0
         0     0     0     1],'absolute',1e-4);
    assertElementsAlmostEqual(r2t(1),...
        [1     0
         0     1],'absolute',1e-4);
    
%    rotx                       - RM for rotation about X-axis
function Test_rotx
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
function Test_roty
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
function Test_rotz
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
function Test_rpy2r
    %Unit test for rpy2r with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(rpy2r(.1, .2, .3),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752],'absolute',1e-4);
    %Unit test for rpy2r with variables ([.1, .2, .3] )
    assertElementsAlmostEqual(rpy2r([.1, .2, .3]),...
        [0.9363   -0.2896    0.1987
         0.3130    0.9447   -0.0978
        -0.1593    0.1538    0.9752],'absolute',1e-4);
    %Unit test for rpy2r with variables ([0 0 0])
    assertElementsAlmostEqual(rpy2r([0 0 0]),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);
    %Unit test for rpy2r with variables ([.1, .2, .3; .4, .5, .6])
    expected_out(:,:,1) = [0.9363   -0.2896    0.1987
                    0.3130    0.9447   -0.0978
                   -0.1593    0.1538    0.9752];
    expected_out(:,:,2) = [0.7243   -0.4955    0.4794
                           0.6742    0.6548   -0.3417
                          -0.1446    0.5707    0.8083];
    assertElementsAlmostEqual(rpy2r([.1, .2, .3; .4, .5, .6]),...
        expected_out,'absolute',1e-4);
    
    
        
%    rpy2tr                     - roll/pitch/yaw angles to HT
function Test_rpy2tr
    %Unit test for rpy2tr with variables (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(rpy2tr(.1, .2, .3),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4);
    %Unit test for rpy2tr with variables ([.1, .2, .3] )
    assertElementsAlmostEqual(rpy2tr([.1, .2, .3]),...
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
function Test_t2r
    %Unit test for r2t with variables eul2tr([.1, .2, .3])
    assertElementsAlmostEqual(t2r(eul2tr( [.1, .2, .3])),...
        [0.9021   -0.3836    0.1977
         0.3875    0.9216    0.0198
        -0.1898    0.0587    0.9801],'absolute',1e-4);
    %Unit test for r2t with variables (0)
    assertElementsAlmostEqual(t2r(1),...
        [],'absolute',1e-4);
    
%    tr2angvec                  - HT/RM to angle/vector form
% CHECK OUTPUT OF THIS FUNCTION!!!!!!!!!!!!!!!!!!!!!!
function Test_tr2angvec
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
    assertExceptionThrown(@()tr2angvec(1),'MATLAB:badsubscript');
    
%    tr2eul                     - HT/RM to Euler angles
function Test_tr2eul
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2eul(eul2tr([.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absolute',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2eul(eul2tr([0 0 0])),...
        [0 0 0],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()tr2eul(1),'MATLAB:badsubscript');
    
%    tr2rpy                     - HT/RM to roll/pitch/yaw angles
function Test_tr2rpy
    %Unit test for rpy2r with variables [.1, .2, .3]
     assertElementsAlmostEqual(tr2rpy(rpy2tr( [.1, .2, .3])),...
        [0.1000    0.2000    0.3000],'absolute',1e-4);
    %Unit test for tr2eul with variables eul2tr( [.1, .2, .3] )
    assertElementsAlmostEqual(tr2rpy(eul2tr([0 0 0])),...
        [0 0 0],'absolute',1e-4);
    %test for scalar input
    assertExceptionThrown(@()tr2rpy(1),'MATLAB:badsubscript');
    
%    transl                     - set or extract the translational component of HT
function Test_transl
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
function Test_trnorm
    %Unit test for oa2r with variables tr matrix generated with oa2tr
    assertElementsAlmostEqual(trnorm(rpy2tr( [.1, .2, .3])),...
        [0.9363   -0.2896    0.1987         0
         0.3130    0.9447   -0.0978         0
        -0.1593    0.1538    0.9752         0
              0         0         0    1.0000],'absolute',1e-4); 
    %test for scalar input
    assertExceptionThrown(@()trnorm(),'MATLAB:inputArgUndefined');    
          
     
%    trotx                      - HT for rotation about X-axis
function Test_trotx
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
function Test_troty
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
function Test_trotz
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


