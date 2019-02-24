%% This is for testing the Differential Motion functions in the robotics Toolbox
function tests = TransformationsTest
  tests = functiontests(localfunctions);
end

%%    skew                       - vector to skew symmetric matrix
function skew_test(tc)
    
    %% 2D case
        verifyEqual(tc, skew(2),...
            [0 -2; 2 0],'absTol',1e-4);
    
    %% 3D case
    
    % test row and column vectors
    verifyEqual(tc, skew([1, 2, 3]'),...
            [     0    -3     2
                 3     0    -1
                -2     1     0],'absTol',1e-4);
    verifyEqual(tc, skew([1, 2, 3]),...
            [     0    -3     2
                 3     0    -1
                -2     1     0],'absTol',1e-4);

end

         
%%    vex                        - skew symmetric matrix to vector
function vex_test(tc)
    %% 2D case
   verifyEqual(tc, vex([0 -2; 2 0]), 2, 'absTol',1e-4);
   
    %% 3D case
    verifyEqual(tc, vex([0, -3, 2; 3, 0, -1; -2, 1, 0]),...
            [1; 2; 3],'absTol',1e-4);
    % unit testing vex with 3x3 skew matrix
    verifyEqual(tc, vex([0 0 0;0 0 0;0 0 0]),...
            [0; 0; 0],'absTol',1e-4);

    verifyError(tc, @()vex(1),'SMTB:vex:badarg');
    verifyError(tc, @()vex(zeros(4,4)),'SMTB:vex:badarg');

    % ---------------------------------------------------------------------
    %    wtrans                     - transform wrench between frames
    % does not exist!!! need to find this function
    %----------------------------------------------------------------------
end

%%    skewa                       - augmented vector to skew symmetric matrix
function skewa_test(tc)
    
    %% 2D case
        verifyEqual(tc, skewa([1 2 3]),...
            [0 -3 1; 3 0 2; 0 0 0],'absTol',1e-4);
    
    %% 3D case
    
    % test row and column vectors
    verifyEqual(tc, skewa([1, 2, 3, 4, 5, 6]'),...
            [0 -6 5 1; 6 0 -4 2; -5 4 0 3; 0 0 0 0],'absTol',1e-4);
    verifyEqual(tc, skewa([1, 2, 3, 4, 5, 6]),...
            [0 -6 5 1; 6 0 -4 2; -5 4 0 3; 0 0 0 0],'absTol',1e-4);

    verifyError(tc, @()skewa(1),'SMTB:skewa:badarg');

end

         
%%    vexa                        - augmented skew symmetric matrix to vector
function vexa_test(tc)
    %% 2D case
   verifyEqual(tc, vexa([0 -3 1; 3 0 2; 0 0 0]), [1 2 3]', 'absTol',1e-4);
   
    %% 3D case
    verifyEqual(tc, vexa([0 -6 5 1; 6 0 -4 2; -5 4 0 3; 0 0 0 0]),...
            [1 2 3 4 5 6]','absTol',1e-4);
    
    verifyError(tc, @()vexa(1),'SMTB:vexa:badarg');

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
    verifyEqual(testCase, delta2tr([0 0 0 0 0 0]), eye(4,4),'absTol',1e-4);
    %test with scaler input 
    verifyError(testCase, @()delta2tr(1),'MATLAB:badsubscript');
end

 
%    eul2jac                    - Euler angles to Jacobian
function eul2jac_test(tc)
    % check it works for simple cases    
    verifyEqual(tc, eul2jac(0, 0, 0), [0 0 0; 0 1 0; 1 0 1]);
    verifyEqual(tc, eul2jac( [0, 0, 0]), [0 0 0; 0 1 0; 1 0 1]);

    eul = [0.2 0.3 0.4];

        
    % check complex case
    verifyEqual(tc, eul2jac( eul(1), eul(2), eul(3)), eul2jac(eul));

    
     %Testing with a scalar number input 
    verifyError(tc, @()eul2jac(1),'SMTB:eul2jac:badarg');
     
     % test Jacobian against numerical approximation 
     dth = 1e-6;
     

         R0 = eul2r(eul);
         R1 = eul2r(eul + dth*[1 0 0]);
         R2 = eul2r(eul + dth*[0 1 0]);
         R3 = eul2r(eul + dth*[0 0 1]);
         
         JJ = [ vex((R1-R0)*R0')  vex((R2-R0)*R0')  vex((R3-R0)*R0')] / dth;
         verifyEqual(tc, JJ, eul2jac(eul), 'absTol',1e-4)

    
end

%    rpy2jac                    - RPY angles to Jacobian
function rpy2jac_test(tc)
    % check it works for simple cases
    verifyEqual(tc, rpy2jac(0, 0, 0), eye(3,3));
    verifyEqual(tc, rpy2jac( [0, 0, 0]), eye(3,3));
    
    % check switches work
    verifyEqual(tc, rpy2jac( [0, 0, 0], 'xyz'), [0 0 1; 0 1 0; 1 0 0]);
    verifyEqual(tc, rpy2jac( [0, 0, 0], 'zyx'), eye(3,3));
    verifyEqual(tc, rpy2jac( [0, 0, 0], 'yxz'), [0 1 0; 0 0 1; 1 0 0]);
    
    rpy = [0.2 0.3 0.4];
    
    % check default
    verifyEqual(tc, rpy2jac(rpy), rpy2jac(rpy, 'zyx') );
    
    
    
    % check complex case
    verifyEqual(tc, rpy2jac( rpy(1), rpy(2), rpy(3)), rpy2jac(rpy));
    
    
    %Testing with a scalar number input
    verifyError(tc, @()rpy2jac(1),'SMTB:rpy2jac:badarg');
    
    % test Jacobian against numerical approximation for 3 different orders
    dth = 1e-6;
    
    for oo = {'xyz', 'zyx', 'yxz'}
        order = oo{1};
        R0 = rpy2r(rpy, order);
        R1 = rpy2r(rpy + dth*[1 0 0], order);
        R2 = rpy2r(rpy + dth*[0 1 0], order);
        R3 = rpy2r(rpy + dth*[0 0 1], order);
        
        JJ = [ vex((R1-R0)*R0')  vex((R2-R0)*R0')  vex((R3-R0)*R0')] / dth;
        verifyEqual(tc, JJ, rpy2jac(rpy, order), 'absTol',1e-4)
    end
    
end
    


%    tr2delta                   - HT to differential motion vector
function tr2delta_test(tc)
    % unit testing tr2delta with a tr matrix
    verifyEqual(tc, tr2delta( transl(0.1, 0.2, 0.3) ),...
        [0.1000, 0.2000, 0.3000, 0, 0, 0]','absTol',1e-4);
    verifyEqual(tc, tr2delta( transl(0.1, 0.2, 0.3), transl(0.2, 0.4, 0.6) ), ...
        [0.1000, 0.2000, 0.3000, 0, 0, 0]','absTol',1e-4);
    verifyEqual(tc, tr2delta( trotx(0.001) ), ...
        [0,0,0, 0.001,0,0]','absTol',1e-4);
    verifyEqual(tc, tr2delta( troty(0.001) ), ...
        [0,0,0, 0,0.001,0]','absTol',1e-4);
    verifyEqual(tc, tr2delta( trotz(0.001) ), ...
        [0,0,0, 0,0,0.001]','absTol',1e-4);
    verifyEqual(tc, tr2delta( trotx(0.001), trotx(0.002) ), ...
        [0,0,0, 0.001,0,0]','absTol',1e-4);
    
    %Testing with a scalar number input
    verifyError(tc, @()tr2delta(1),'SMTB:tr2delta:badarg');
    verifyError(tc, @()tr2delta( ones(3,3) ),'SMTB:tr2delta:badarg');

end
    
%    tr2jac                     - HT to Jacobian
function tr2jac_test(tc)
    % unit testing tr2jac with homogeneous transform
    T = transl(1,2,3);
    [R,t] = tr2rt(T);
    
    verifyEqual(tc, tr2jac(T), [R' zeros(3,3); zeros(3,3) R']);
    verifyEqual(tc, tr2jac(T, 'samebody'), [R' (skew(t)*R)'; zeros(3,3) R']);
    
     T = transl(1,2,3) * trotx(pi/2) * trotz(pi/2);
    [R,t] = tr2rt(T);
    
    verifyEqual(tc, tr2jac(T), [R' zeros(3,3); zeros(3,3) R']);
    verifyEqual(tc, tr2jac(T, 'samebody'), [R' (skew(t)*R)'; zeros(3,3) R']);   
    

    % test with scalar value
    verifyError(tc, @()tr2jac(1),'SMTB:t2r:badarg');
end
     

function wtrans_test(tc)
    
    v = [1 2 3 4 5 6]';
    
    tc.verifyEqual( wtrans(eye(4,4), v), v, 'abstol', 1e-10 );
    tc.verifyEqual( wtrans(trotx(pi/2), v), [1 3 -2 4 6 -5]', 'abstol', 1e-10 );       
    tc.verifyEqual( wtrans(troty(pi/2), v), [-3 2 1 -6 5 4]', 'abstol', 1e-10 );       
    tc.verifyEqual( wtrans(trotz(pi/2), v), [2 -1 3 5 -4 6]', 'abstol', 1e-10 ); 
end

