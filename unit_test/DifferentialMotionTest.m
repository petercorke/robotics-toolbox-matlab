%% This is for testing the Differential Motion functions in the robotics Toolbox
function tests = TransformationsTest
  tests = functiontests(localfunctions);
end

%% Differential motion
%    delta2tr                   - differential motion vector to HT
function delta2tr_test(testCase)
    %test with standard numbers  
    verifyEqual(testCase, delta2tr([0.1 0.2 0.3 0.4 0.5 0.6]),...
    [1.0000   -0.6000    0.5000    0.1000
    0.6000    1.0000   -0.4000    0.2000
   -0.5000    0.4000    1.0000    0.3000
         0         0         0    1.0000],'absTol',1e-4);
    %test with zeros
    verifyEqual(testCase, delta2tr([0 0 0 0 0 0]),...
    [1     0     0     0
     0     1     0     0
     0     0     1     0
     0     0     0     1],'absTol',1e-4);
    %test with scaler input 
    verifyError(testCase, @()delta2tr(1),'MATLAB:badsubscript');
end
 
%    eul2jac                    - Euler angles to Jacobian
function eul2jac_test(testCase)
    % unit testing eul2jac with variable (0.1, 0.2, 0.3)
    verifyEqual(testCase, eul2jac(0.1, 0.2, 0.3),...
    [    0   -0.0998    0.1977
         0    0.9950    0.0198
    1.0000         0    0.9801],'absTol',1e-4);
    % unit testing eul2jac with variable ([.1, .2, .3; .4, .5, .6])
    verifyEqual(testCase, eul2jac([.1, .2, .3; .4, .5, .6]),...
        [     0   -0.0998    0.3875
         0    0.9950    0.0389
    1.0000         0    0.9211],'absTol',1e-4);
    % unit testing eul2jac with variable (0.1, 0.2, 0.3)
    verifyEqual(testCase, eul2jac(0, 0, 0),...
        [0     0     0
         0     1     0
         1     0     1],'absTol',1e-4);
    verifyError(testCase, @()eul2jac(1),'RTB:eul2jac:badarg');
end

%    rpy2jac                    - RPY angles to Jacobian
function rpy2jac_test(testCase)
    % unit testing rpy2jac with variable (.1,.2,.3)
    verifyEqual(testCase, rpy2jac(.1, .2, .3),...
        [1.0000         0    0.1987
              0    0.9950   -0.0978
              0    0.0998    0.9752],'absTol',1e-4);
    % unit testing rpy2jac with variable ([.1, .2, .3])
    verifyEqual(testCase, rpy2jac([.1, .2, .3]),...
        [1.0000         0    0.1987
              0    0.9950   -0.0978
              0    0.0998    0.9752],'absTol',1e-4);
    % unit testing rpy2jac with variable (0,0,0)
    verifyEqual(testCase, rpy2jac(0, 0, 0),...
        [1     0     0
         0     1     0
         0     0     1],'absTol',1e-4);
     %Testing with a scalar number input 
     verifyError(testCase, @()rpy2jac(1),'RTB:rpy2jac:badarg');
end
    

%    skew                       - vector to skew symmetric matrix
function skew_test(testCase)
    % unit testing the function skew(testCase) with a vector matrix mat([.1, .2, .3])
    verifyEqual(testCase, skew([.1, .2, .3]),...
            [0   -0.3000    0.2000
        0.3000         0   -0.1000
       -0.2000    0.1000         0],'absTol',1e-4);
   % unit testing the function skew(testCase) with a vertical vector matrix mat([.1, .2, .3])
    verifyEqual(testCase, skew([.1; .2; .3]),...
            [0   -0.3000    0.2000
        0.3000         0   -0.1000
       -0.2000    0.1000         0],'absTol',1e-4);
     % unit testing the function skew(testCase) with zero values
    verifyEqual(testCase, skew([0 0 0]),...
            [0     0     0
             0     0     0
             0     0     0],'absTol',1e-4);
     %Testing with a scalar number input 
     verifyEqual(testCase, skew(1),...
            [0    -1
             1     0],'absTol',1e-4);
end
    
%    tr2delta                   - HT to differential motion vector
function tr2delta_test(testCase)
    % unit tessting tr2delta with a tr matrix
    verifyEqual(testCase, tr2delta([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16]),...
            [4.0000
             8.0000
            12.0000
             1.5000
            -3.0000
             1.5000],'absTol',1e-4);
    % Unit testing tr2delta with two homogeneous transformations
    verifyEqual(testCase, tr2delta([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16],...
        [16 15 14 13; 12 11 10 9;8 7 6 5;4 3 2 1]),...
            [9
             1
            -7
            -102
             204
            -102],'absTol',1e-4);
    % unit tessting tr2delta with a tr matrix of zeros
    verifyEqual(testCase, tr2delta([0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]),...
        [0
         0
         0
         0
         0
         0],'absTol',1e-4);
    %Testing with a scalar number input 
    verifyError(testCase, @()tr2delta(1),'RTB:t2r:badarg');
end
    
%    tr2jac                     - HT to Jacobian
function tr2jac_test(testCase)
    % unit testing tr2jac with homogeneous transform
    verifyEqual(testCase, tr2jac([1 2 3 4;5 6 7 8;9 10 11 12;13 14 15 16]),...
            [1     5     9    12   -24    12
             2     6    10     8   -16     8
             3     7    11     4    -8     4
             0     0     0     1     5     9
             0     0     0     2     6    10
             0     0     0     3     7    11],'absTol',1e-4);
    % unit testing tr2jac with homogeneous transform of zeros
    verifyEqual(testCase, tr2jac([0 0 0 0;0 0 0 0;0 0 0 0;0 0 0 0]),...
            [0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0
             0     0     0     0     0     0],'absTol',1e-4);
    % test with scalar value
    verifyError(testCase, @()tr2jac(1),'RTB:t2r:badarg');
end
     
         
%    vex                        - skew symmetric matrix to vector
function vex_test(testCase)
    % unit testing vex with homogeneous transform
    verifyEqual(testCase, vex([0 -0.3 0.2;0.3 0 -0.1;-0.2 0.1 0]),...
            [0.1000
             0.2000
             0.3000],'absTol',1e-4);
    % unit testing vex with homogeneous transform
    verifyEqual(testCase, vex([0 0 0;0 0 0;0 0 0]),...
            [0
             0
             0],'absTol',1e-4);
    verifyError(testCase, @()vex(1),'RTB:vex:badarg');
    % ---------------------------------------------------------------------
    %    wtrans                     - transform wrench between frames
    % does not exist!!! need to find this function
    %----------------------------------------------------------------------
end
