%% This is for testing the Trajectory Generation functions in the robotics Toolbox
function tests = TrajectoryTest
  tests = functiontests(localfunctions);
end

  %% Trajectory Generation
%    tpoly                      - 1D polynomial trajectory
function tpoly_test(testCase)
    % unit testing tpoly 
    s1 = 1;
    s2 = 2;
    T = [0.1 0.2 0.3 0.4 0.5];
    V = 0.5;
    %Testing tpoly with N = 5 
    [Q Qd Qdd] = tpoly(s1,s2,5);
    verifyEqual(testCase, Q,...
            [1.0000
             1.1035
             1.5000
             1.8965
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0
             0.2637
             0.4688
             0.2637
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [     0
             0.3516
            -0.0000
            -0.3516
            -0.0000],'absTol',1e-4);
    %Testing tpoly with T
    [Q Qd Qdd] = tpoly(s1,s2,T);
    verifyEqual(testCase, Q,...
            [1.0579
             1.3174
             1.6826
             1.9421
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [1.5360
             3.4560
             3.4560
             1.5360
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [23.0400
             11.5200
            -11.5200
            -23.0400
                   0],'absTol',1e-4);
    %Testing tpoly with N = 5 and a constant velocity of V 
    [Q Qd Qdd] = tpoly(s1,s2,5,V);
    verifyEqual(testCase, Q,...
            [1.0000
             1.4727
             1.8125
             1.9727
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [0.5000
             0.4219
             0.2500
             0.0781
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [     0
            -0.1406
            -0.1875
            -0.1406
                  0],'absTol',1e-4);
end
        
%    lspb                       - 1D trapezoidal trajectory
function lspb_test(testCase)
    % unit testing lspb 
    s1 = 1;
    s2 = 2;
    T = [0.1 0.2 0.3 0.4 0.5];
    V = 0.5;
    %Testing lspb with N = 5 
    [Q Qd Qdd] = lspb(s1,s2,5);
    verifyEqual(testCase, Q,...
            [1.0000
             1.1406
             1.5000
             1.8594
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0
             0.2813
             0.3750
             0.2813
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [0.2813
             0.2813
                  0
            -0.2813
            -0.2813],'absTol',1e-4);
    %Testing lspb with T
    [Q Qd Qdd] = lspb(s1,s2,T);
    verifyEqual(testCase, Q,...
            [1.0900
             1.3500
             1.6500
             1.9100
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [1.8000
             3.0000
             3.0000
             1.8000
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [18
              0
              0
            -18
            -18],'absTol',1e-4);
    %Testing lspb with N = 5 and a constant velocity of V 
    [Q Qd Qdd] = lspb(s1,s2,5,V);
    verifyEqual(testCase, Q,...
            [1.0000
             1.1250
             1.5000
             1.8750
             2.0000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0
             0.2500
             0.5000
             0.2500
                  0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [0.2500
             0.2500
             0.2500
            -0.2500
            -0.2500],'absTol',1e-4);
end
        
%    ctraj                      - Cartesian trajectory
function ctraj_test(testCase)
    % unit testing ctraj with T0 and T1 and N
    expected_out(:,:,1) = [1     0     0     1
                           0     1     0     1
                           0     0     1     1
                           0     0     0     1];
    expected_out(:,:,2) = [1.0000         0         0    1.5000
                                0    1.0000         0    1.5000
                                0         0    1.0000    1.5000
                                0         0         0    1.0000];
    expected_out(:,:,3) = [1     0     0     2
                           0     1     0     2
                           0     0     1     2
                           0     0     0     1];
    verifyEqual(testCase, ctraj(transl([1 1 1]),transl([2 2 2]),3),...
            expected_out,'absTol',1e-4);
    % unit testing ctraj with T0 and T1 and S(i)
    expected_out(:,:,1) = [1.0000         0         0    1.1000
                                0    1.0000         0    1.1000
                                0         0    1.0000    1.1000
                                0         0         0    1.0000];
    expected_out(:,:,2) = [1.0000         0         0    1.5000
                                0    1.0000         0    1.5000
                                0         0    1.0000    1.5000
                                0         0         0    1.0000];
    expected_out(:,:,3) = [1.0000         0         0    1.9000
                                0    1.0000         0    1.9000
                                0         0    1.0000    1.9000
                                0         0         0    1.0000];
    verifyEqual(testCase, ctraj(transl([1 1 1]),transl([2 2 2]),[0.1 0.5 0.9]),...
            expected_out,'absTol',1e-4);
end
            
%    jtraj                      - joint space trajectory
function jtraj_test(testCase)
    % unit testing jtraj with 
    q1 = [0.1 0.2 0.3 0.4 0.5];
    q2 = [0.5 0.4 0.3 0.2 0.1];
    T = [0.1 0.2 0.3 0.4 0.5];
    %Testing jtraj with N = 5 
    [Q Qd Qdd] = jtraj(q1,q2,5);
    verifyEqual(testCase, Q,...
            [0.1000    0.2000    0.3000    0.4000    0.5000
             0.1414    0.2207    0.3000    0.3793    0.4586
             0.3000    0.3000    0.3000    0.3000    0.3000
             0.4586    0.3793    0.3000    0.2207    0.1414
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [0         0         0         0         0
             0.4219    0.2109         0   -0.2109   -0.4219
             0.7500    0.3750         0   -0.3750   -0.7500
             0.4219    0.2109         0   -0.2109   -0.4219
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [     0         0         0         0         0
             2.2500    1.1250         0   -1.1250   -2.2500
                  0         0         0         0         0
            -2.2500   -1.1250         0    1.1250    2.2500
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
    %Testing jtraj where T is a matrix 
    [Q Qd Qdd] = jtraj(q1,q2,T);
    verifyEqual(testCase, Q,...
            [0.1232    0.2116    0.3000    0.3884    0.4768
             0.2270    0.2635    0.3000    0.3365    0.3730
             0.3730    0.3365    0.3000    0.2635    0.2270
             0.4768    0.3884    0.3000    0.2116    0.1232
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [0.6144    0.3072         0   -0.3072   -0.6144
             1.3824    0.6912         0   -0.6912   -1.3824
             1.3824    0.6912         0   -0.6912   -1.3824
             0.6144    0.3072         0   -0.3072   -0.6144
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [9.2160    4.6080         0   -4.6080   -9.2160
             4.6080    2.3040         0   -2.3040   -4.6080
            -4.6080   -2.3040         0    2.3040    4.6080
            -9.2160   -4.6080         0    4.6080    9.2160
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
     % testing with all final joint velocites  = q2 &q1 
     [Q Qd Qdd] = jtraj(q1,q2,T,q2,q1);
    verifyEqual(testCase, Q,...
            [0.1630    0.2422    0.3213    0.4004    0.4796
             0.2691    0.2908    0.3124    0.3340    0.3556
             0.3904    0.3390    0.2876    0.2362    0.1848
             0.4741    0.3764    0.2787    0.1810    0.0833
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [0.8424    0.4560    0.0696   -0.3168   -0.7032
             1.2232    0.5024   -0.2184   -0.9392   -1.6600
             1.1048    0.4432   -0.2184   -0.8800   -1.5416
             0.5256    0.2976    0.0696   -0.1584   -0.3864
             0.1000    0.2000    0.3000    0.4000    0.5000],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [4.9920    0.7680   -3.4560   -7.6800  -11.9040
             1.7280    0.0000   -1.7280   -3.4560   -5.1840
            -4.0320   -1.1520    1.7280    4.6080    7.4880
            -6.5280   -1.5360    3.4560    8.4480   13.4400
             0.0000    0.0000         0   -0.0000    0.0000],'absTol',1e-4);
end
     
         
%    mtraj                      - multi-axis trajectory for arbitrary function        
function mtraj_test(testCase)
% unit testing lspb 
    q1 = [0.1 0.2 0.3 0.4 0.5];
    q2 = [0.5 0.4 0.3 0.2 0.1];
    T = [0.1 0.2 0.3 0.4 ];
    %Testing lspb with N = 5 
    [Q Qd Qdd] = mtraj(@lspb,q1,q2,5);
    verifyEqual(testCase, Q,...
            [0.1000    0.2000    0.3000    0.4000    0.5000
             0.1563    0.2281    0.3000    0.3719    0.4437
             0.3000    0.3000    0.3000    0.3000    0.3000
             0.4438    0.3719    0.3000    0.2281    0.1562
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0         0         0         0         0
             0.1125    0.0563         0   -0.0563   -0.1125
             0.1500    0.0750         0   -0.0750   -0.1500
             0.1125    0.0562         0   -0.0562   -0.1125
                  0         0         0         0         0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [0.1125    0.0563         0   -0.0563   -0.1125
             0.1125    0.0563         0   -0.0563   -0.1125
                  0         0         0         0         0
            -0.1125   -0.0563         0    0.0563    0.1125
            -0.1125   -0.0563         0    0.0563    0.1125],'absTol',1e-4);
    %Testing lspb with T 
    [Q Qd Qdd] = mtraj(@lspb,q1,q2,T);
    verifyEqual(testCase, Q,...
            [0.1000    0.2000    0.3000    0.4000    0.5000
             0.2000    0.2500    0.3000    0.3500    0.4000
             0.4000    0.3500    0.3000    0.2500    0.2000
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0         0         0         0         0
             0.2000    0.1000         0   -0.1000   -0.2000
             0.2000    0.1000         0   -0.1000   -0.2000
                  0         0         0         0         0],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [0.2000    0.1000         0   -0.1000   -0.2000
             0.2000    0.1000         0   -0.1000   -0.2000
            -0.2000   -0.1000         0    0.1000    0.2000
            -0.2000   -0.1000         0    0.1000    0.2000],'absTol',1e-4);
    %Testing tpoly with T 
    [Q Qd Qdd] = mtraj(@tpoly,q1,q2,T);
    verifyEqual(testCase, Q,...
            [0.1000    0.2000    0.3000    0.4000    0.5000
             0.1840    0.2420    0.3000    0.3580    0.4160
             0.4160    0.3580    0.3000    0.2420    0.1840
             0.5000    0.4000    0.3000    0.2000    0.1000],'absTol',1e-4);
    verifyEqual(testCase, Qd,...
            [     0         0         0         0         0
             0.1975    0.0988         0   -0.0988   -0.1975
             0.1975    0.0988         0   -0.0988   -0.1975
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
    verifyEqual(testCase, Qdd,...
            [     0         0         0         0         0
             0.1975    0.0988         0   -0.0988   -0.1975
            -0.1975   -0.0988         0    0.0988    0.1975
             0.0000    0.0000         0   -0.0000   -0.0000],'absTol',1e-4);
end

%    mstraj                     - multi-axis multi-segment trajectory
function mstraj_test(testCase)
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
     verifyEqual(testCase, out,expected_out ,'absTol',1e-4);

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
     verifyEqual(testCase, out,expected_out ,'absTol',1e-4);
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
     verifyEqual(testCase, out,expected_out ,'absTol',1e-4);
end


    
%    trinterp                   - interpolate HT s
function trinterp_test(testCase)
    % Unit testing trinterp with two translation matrices. 
    T1 = [1.0000         0         0    0.4000
               0   -1.0000   -0.0000    0.2000
               0    0.0000   -1.0000         0
               0         0         0    1.0000];
    T2 = [1.0000         0         0    0.4000
               0    1.0000         0    0.2000
               0         0    1.0000         0
               0         0         0    1.0000];
    S = [0 0.5 1];
    expected_out(:,:,1)= [1.0000         0         0    0.4000
                               0   -1.0000         0    0.2000
                               0         0   -1.0000         0
                               0         0         0    1.0000];
    expected_out(:,:,2)= [1.0000         0         0    0.4000
                               0    0.0000   -1.0000    0.2000
                               0    1.0000    0.0000         0
                               0         0         0    1.0000];
    expected_out(:,:,3)= [1.0000         0         0    0.4000
                               0    1.0000         0    0.2000
                               0         0    1.0000         0
                               0         0         0    1.0000];
    verifyEqual(testCase, trinterp(T1,T2,S),...
            expected_out, 'absTol',1e-4);
    % Unit testing trinterp with one rotation matrix. 
    T1 = [1.0000         0         0    0.4000
               0   -1.0000   -0.0000    0.2000
               0    0.0000   -1.0000         0
               0         0         0    1.0000];
    S = [0 0.5 1];
    expected_out(:,:,1)= [1     0     0     0
                          0     1     0     0
                          0     0     1     0
                          0     0     0     1];
    expected_out(:,:,2)= [1.0000         0         0    0.2000
                               0   -0.0000   -1.0000    0.1000
                               0    1.0000   -0.0000         0
                               0         0         0    1.0000];
    expected_out(:,:,3)= [1.0000         0         0    0.4000
                               0   -1.0000         0    0.2000
                               0         0   -1.0000         0
                               0         0         0    1.0000];
    verifyEqual(testCase, trinterp(T1,S),...
            expected_out, 'absTol',1e-4);
end
