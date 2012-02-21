%% This is for testing the Differential Motion functions in the robotics Toolbox
function test_suite = TestRobotToolboxDifferentialMotion
  initTestSuite;
  
%% Differential motion
%    delta2tr                   - differential motion vector to HT
function Test_delta2tr
    %test with standard numbers  
    assertElementsAlmostEqual(delta2tr([0.1 0.2 0.3 0.4 0.5 0.6]),...
    [1.0000   -0.6000    0.5000    0.1000
    0.6000    1.0000   -0.4000    0.2000
   -0.5000    0.4000    1.0000    0.3000
         0         0         0    1.0000],'absolute',1e-4);
    %test with zeros
    assertElementsAlmostEqual(delta2tr([0 0 0 0 0 0]),...
    [1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1],'absolute',1e-4);
    %test with scaler input 
    assertExceptionThrown(@()delta2tr(1),'MATLAB:badsubscript');
 
%    eul2jac                    - Euler angles to Jacobian
function Test_eul2jac
    % unit testing eul2jac with variable (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(eul2jac(0.1, 0.2, 0.3),...
        [0.1898   -0.2955         0
         0.0587    0.9553         0
         0.9801         0    1.0000],'absolute',1e-4);
    % unit testing eul2jac with variable ([.1, .2, .3; .4, .5, .6])
    assertElementsAlmostEqual(eul2jac([.1, .2, .3; .4, .5, .6]),...
        [0.3817   -0.1987         0
         0.0774    0.9801         0
         0.9211         0    1.0000],'absolute',1e-4);
    % unit testing eul2jac with variable (0.1, 0.2, 0.3)
    assertElementsAlmostEqual(eul2jac(0, 0, 0),...
        [0     0     0
         0     1     0
         1     0     1],'absolute',1e-4);
    assertExceptionThrown(@()eul2jac(1),'MATLAB:inputArgUndefined');

%    rpy2jac                    - RPY angles to Jacobian
function Test_rpy2jac
    % unit testing rpy2jac with variable (.1,.2,.3)
    assertElementsAlmostEqual(rpy2jac(.1, .2, .3),...
        [1.0000         0    0.1987
              0    0.9950   -0.0978
              0    0.0998    0.9752],'absolute',1e-4);
    % unit testing rpy2jac with variable ([.1, .2, .3])
    assertElementsAlmostEqual(rpy2jac([.1, .2, .3]),...
        [1.0000         0    0.1987
              0    0.9950   -0.0978
              0    0.0998    0.9752],'absolute',1e-4);
    % unit testing rpy2jac with variable (0,0,0)
    assertElementsAlmostEqual(rpy2jac(0, 0, 0),...
        [1     0     0
         0     1     0
         0     0     1],'absolute',1e-4);
     %Testing with a scalar number input 
     assertExceptionThrown(@()rpy2jac(1),'MATLAB:inputArgUndefined');
    

%    skew                       - vector to skew symmetric matrix
function Test_skew
    % unit testing the function skew with a vector matrix mat([.1, .2, .3])
    assertElementsAlmostEqual(skew([.1, .2, .3]),...
            [0   -0.3000    0.2000
        0.3000         0   -0.1000
       -0.2000    0.1000         0],'absolute',1e-4);
   % unit testing the function skew with a vertical vector matrix mat([.1, .2, .3])
    assertElementsAlmostEqual(skew([.1; .2; .3]),...
            [0   -0.3000    0.2000
        0.3000         0   -0.1000
       -0.2000    0.1000         0],'absolute',1e-4);
     % unit testing the function skew with zero values
    assertElementsAlmostEqual(skew([0 0 0]),...
            [0     0     0
             0     0     0
             0     0     0],'absolute',1e-4);
     %Testing with a scalar number input 
     assertElementsAlmostEqual(skew(1),...
            [0    -1
             1     0],'absolute',1e-4);
    
%    tr2delta                   - HT to differential motion vector
function Test_tr2delta
    % unit tessting tr2delta with a tr matrix
    assertElementsAlmostEqual(tr2delta([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16]),...
            [4.0000
             8.0000
            12.0000
             1.5000
            -3.0000
             1.5000],'absolute',1e-4);
    % Unit testing tr2delta with two homogeneous transformations
    assertElementsAlmostEqual(tr2delta([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16],...
        [16 15 14 13; 12 11 10 9;8 7 6 5;4 3 2 1]),...
            [9
             1
            -7
            -102
             204
            -102],'absolute',1e-4);
    % unit tessting tr2delta with a tr matrix of zeros
    assertElementsAlmostEqual(tr2delta([0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]),...
        [0
         0
         0
         0
         0
         0],'absolute',1e-4);
    %Testing with a scalar number input 
    assertExceptionThrown(@()tr2delta(1),'MATLAB:badsubscript');
    
%    tr2jac                     - HT to Jacobian
function Test_tr2jac
    % unit testing tr2jac with homogeneous transform
    assertElementsAlmostEqual(tr2jac([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16]),...
            [1     5     9    12   -24    12
             2     6    10     8   -16     8
             3     7    11     4    -8     4
             0     0     0     1     5     9
             0     0     0     2     6    10
             0     0     0     3     7    11],'absolute',1e-4);
    % unit testing tr2jac with homogeneous transform of zeros
    assertElementsAlmostEqual(tr2jac([0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]),...
            [0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0],'absolute',1e-4);
    % test with scalar value
    assertExceptionThrown(@()tr2jac(1),'');
     
         
%    vex                        - skew symmetric matrix to vector
function Test_vex
    % unit testing vex with homogeneous transform
    assertElementsAlmostEqual(vex([0 -.3 0.2;0.3 0 -0.1;-0.2 0.1 0]),...
            [0.1000
             0.2000
             0.3000],'absolute',1e-4);
    % unit testing vex with homogeneous transform
    assertElementsAlmostEqual(vex([0 0 0;0 0 0;0 0 0]),...
            [0
             0
             0],'absolute',1e-4);
    assertExceptionThrown(@()vex(1),'');
    % ---------------------------------------------------------------------
    %    wtrans                     - transform wrench between frames
    % does not exist!!! need to find this function
    %----------------------------------------------------------------------
