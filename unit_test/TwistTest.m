function tests = TwistTest
  tests = functiontests(localfunctions);
  
  clc
end

function twist2d_test(tc)
    %%2D twists
    
    % check basics work
    s = [1 2 3];
    tw = Twist(s);
    verifyEqual(tc, tw.S, s', 'absTol', 1e-6);
    verifyEqual(tc, tw.v', s(1:2), 'absTol', 1e-6);
    verifyEqual(tc, tw.w, s(3), 'absTol', 1e-6);
    verifyEqual(tc, tw.se, [skew(s(3)) [s(1:2)]'; 0 0 0], 'absTol', 1e-6);
end



function operator2d_test(tc)
    % check overloaded *

    tw = Twist([1 2 3]);
    tw2 = Twist([4 6 5]);
    
    both = tw * tw2;
    
    Tboth = tw.T * tw2.T;
    verifyEqual(tc, both.T, Tboth, 'absTol', 1e-6);
    
    % check rotational twist
    tw = Twist('R', [1 2]);
    verifyEqual(tc, tw.S, [2 -1 1]', 'absTol', 1e-6);
    
    % check prismatic twist
    tw = Twist('P', [2 3]);
    verifyEqual(tc, tw.S, [unit([2 3]) 0]', 'absTol', 1e-6);
    tw = Twist('T', [2 3]);
    verifyEqual(tc, tw.S, [unit([2 3]) 0]', 'absTol', 1e-6);
    
    % check twist from SE(2)
    tw = Twist( trot2(0) );
    verifyEqual(tc, tw.S, [0 0 0]', 'absTol', 1e-6);
    tw = Twist( trot2(pi/2) );
    verifyEqual(tc, tw.S, [0 0 pi/2]', 'absTol', 1e-6);    
    tw = Twist( SE2(1,2,0) );
    verifyEqual(tc, tw.S, [1 2 0]', 'absTol', 1e-6);
    tw = Twist( SE2(1,2,pi/2) );
    verifyEqual(tc, tw.S, [3*pi/4 pi/4 pi/2]', 'absTol', 1e-6);

    % test expm and T
    verifyEqual(tc, tw.T, trexp2(tw.S), 'absTol', 1e-6);
    verifyEqual(tc, tw.exp.double, trexp2(tw.S), 'absTol', 1e-6);

    tw = Twist('R', [1 2]);
    verifyEqual(tc, tw.T(pi/2), [0 -1 3; 1 0 1; 0 0 1], 'absTol', 1e-6);
end

function operator3d_test(tc)
    %% 3D twists
    
    % check basics work
    s = [1 2 3 4 5 6];
    tw = Twist(s);
    verifyEqual(tc, tw.S, s', 'absTol', 1e-6);
    verifyEqual(tc, tw.v', s(1:3), 'absTol', 1e-6);
    verifyEqual(tc, tw.w', s(4:6), 'absTol', 1e-6);
    verifyEqual(tc, tw.se, [skew(s(4:6)) [s(1:3)]'; 0 0 0 0], 'absTol', 1e-6);
    
    
    % check overloaded *
    s2 = [4 6 5 7 9 8];
    tw2 = Twist(s2);
    
    both = tw * tw2;
    
    Tboth = tw.T * tw2.T;
    verifyEqual(tc, double(both.T), double(Tboth), 'absTol', 1e-6);
    
    
    % check rotational twist
    tw = Twist('R', [1 2 3], [0 0 0]);
    verifyEqual(tc, tw.S, [0 0 0 unit([1 2 3])]', 'absTol', 1e-6);
    
    % check prismatic twist
    tw = Twist('P', [1 2 3]);
    verifyEqual(tc, tw.S, [unit([1 2 3]) 0 0 0 ]', 'absTol', 1e-6);
    tw2 = Twist('T', [1 2 3]);
    verifyEqual(tc, tw.S, tw2.S, 'absTol', 1e-6);
    
    % check twist from SE(3)
    tw = Twist( trotx(0) );
    verifyEqual(tc, tw.S, [0 0 0  0 0 0]', 'absTol', 1e-6);
    tw = Twist( trotx(pi/2) );
    verifyEqual(tc, tw.S, [0 0 0  pi/2 0 0]', 'absTol', 1e-6);    
    tw = Twist( troty(pi/2) );
    verifyEqual(tc, tw.S, [0 0 0  0 pi/2 0]', 'absTol', 1e-6);
    tw = Twist( trotz(pi/2) );
    verifyEqual(tc, tw.S, [0 0 0  0 0 pi/2]', 'absTol', 1e-6);
    
    tw = Twist( transl([1 2 3]) );
    verifyEqual(tc, tw.S, [1 2 3  0 0 0]', 'absTol', 1e-6);
    tw = Twist( transl([1 2 3])*troty(pi/2) );
    verifyEqual(tc, tw.S, [-pi/2 2 pi  0 pi/2 0]', 'absTol', 1e-6);

    % test expm and T
    verifyEqual(tc, tw.T, SE3.exp(tw.S).T);
    verifyEqual(tc, tw.exp, SE3.exp(tw.S));
    
    tw = Twist('R', [1 0 0], [0 0 0]);
    verifyEqual(tc, tw.T(pi/2), SE3.Rx(pi/2).T, 'absTol', 1e-6);
    tw = Twist('R', [0 1 0], [0 0 0]);
    verifyEqual(tc, tw.T(pi/2), SE3.Ry(pi/2).T, 'absTol', 1e-6);
    tw = Twist('R', [0 0 1], [0 0 0]);
    verifyEqual(tc, tw.T(pi/2), SE3.Rz(pi/2).T, 'absTol', 1e-6);
end


function char_test(tc)
    % 2d

    tw = Twist([1 2 3]);
    
    s = char(tw);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 1);
    s = char([tw tw tw]);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 3);
    s = char([tw tw tw]');
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 3);
    
    % 3d
    tw = Twist([4 6 5 7 9 8]);
    s = char(tw);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 1);
    s = char([tw tw tw]);
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 3);
    s = char([tw tw tw]');
    tc.verifyClass(s, 'char');
    tc.verifyEqual(size(s,1), 3);
end