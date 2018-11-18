function tests = SE2Test
    tests = functiontests(localfunctions);
end


% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    verifyClass(tc, SE2(), 'SE2');

    %% null
    tc.verifyEqual(SE2().double, eye(3,3));
    
    %% translation only
    t = [1 2];
    tc.verifyEqual(SE2(t).double, transl2(t));
    tc.verifyEqual(SE2(t').double, transl2(t));
    tc.verifyEqual(SE2(t(1), t(2)).double, transl2(t));
    
    %% R
    R = rot2(-pi/2);
    tc.verifyEqual(SE2(R).double, r2t(R));
    
    %% R,t

    tc.verifyEqual(SE2(R,t).double, transl2(t)*r2t(R));
    
    %% T
    T = transl2(1, 2)*trot2(0.3);
    tc.verifyEqual(SE2(T).double, T);
    
    %% x,y,theta
    T = transl2(1, 2)*trot2(0.3);
    tc.verifyEqual(SE2(1, 2, 0.3).double, T);
    tc.verifyEqual(SE2([1, 2, 0.3]).double, T);
    tc.verifyEqual(SE2([1, 2], 0.3).double, T);
        
    %% T
    T = rt2tr(R,t);
    tc.verifyEqual(SE2(T).double, T);
    
    %% copy constructor
    TT = SE2(T);
    tc.verifyEqual(SE2(TT).double, T);
    
    
    %% vectorised versions
    
    T1 = transl2(1,2) * trot2(0.3);
    T2 = transl2(1,-2) * trot2(-0.4);
    
    TT = cat(3, T1, T2, T1, T2);

    tt = SE2(TT);
    tc.verifyEqual(length(tt), size(TT, 3) );
    tc.verifyEqual(tt.T, TT);
    
end

function concat_test(tc)
    x = SE2();
    xx = [x x x x];
    
    tc.verifyClass(xx, 'SE2');
    tc.verifySize(xx, [1 4]);
end

function staticconstructors_test(tc)
    
    %% exponential
    tc.verifyEqual(SE2.exp( skew(0.3) ).R, rot2(0.3), 'AbsTol', 1e-10  );
    
        
    %% exponential
    tc.verifyEqual(SE3.exp(zeros(4,4)).T, eye(4,4), 'AbsTol', 1e-10  );
    t= [1 2 3];
    tc.verifyEqual(SE3.exp([t 0 0 0]').T, transl(t), 'AbsTol', 1e-10  );
end

function isa_test(tc)
    
    verifyTrue(tc, SE2.isa(trot2(0)) );
    verifyFalse(tc, SE2.isa(1) )
end

function resulttype_test(tc)
    
    t = SE2();
    verifyClass(tc, t, 'SE2');
    
    verifyClass(tc, t*t, 'SE2');
    
    verifyClass(tc, t/t, 'SE2');
    
    verifyClass(tc, inv(t), 'SE2');
    end

function inverse_test(tc)    
    
    T1 = transl2(1, 2) * trot2(0.3);
    TT1 = SE2(T1);
    
    % test inverse
    tc.verifyEqual(double(TT1.inv()), inv(T1), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(double(TT1*TT1.inv()), eye(3,3), 'AbsTol', 1e-10  );
    tc.verifyEqual(double(TT1.inv()*TT1), eye(3,3), 'AbsTol', 1e-10  );
    
    % vector case
    tc.verifyEqual(double(TT1.inv()*TT1), eye(3,3), 'AbsTol', 1e-10  );
end

% function uminus_test(tc)
%     R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
%     
%     TT1 = SE3(T1);
% 
%     
%     TM = - TT1;
%     tc.verifyEqual(TM.double, -T1, 'AbsTol', 1e-10  );
% end

function Rt_test(tc)
    
    R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
    
    TT1 = SE3(T1);
    
    tc.verifyEqual(TT1.T, T1, 'AbsTol', 1e-10  );
    tc.verifyEqual(TT1.R, R1, 'AbsTol', 1e-10  );
    tc.verifyEqual(TT1.t, t1, 'AbsTol', 1e-10  );
    
    tc.verifyEqual(TT1.transl, t1', 'AbsTol', 1e-10  );
    TT = [TT1 TT1 TT1];
    tc.verifyEqual(TT.transl, [t1 t1 t1]', 'AbsTol', 1e-10  );
end


function arith_test(tc)
    
    R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
    R2 = rpy2r( randn(1,3) );  t2 = randn(3,1); T2 = rt2tr(R2, t2);
    
    TT1 = SE3(T1);
    TT2 = SE3(T2);

    I = SE3();
    
    
    %% SE3-SE3 product
    % scalar x scalar
    
    tc.verifyEqual(double(TT1*TT2), T1*T2, 'AbsTol', 1e-10  );
    tc.verifyEqual(double(TT2*TT1), T2*T1, 'AbsTol', 1e-10  );
    tc.verifyEqual(double(TT1*I), T1, 'AbsTol', 1e-10  );
    tc.verifyEqual(double(TT2*I), T2, 'AbsTol', 1e-10  );
    
    % vector x vector
    tc.verifyEqual([TT1 TT1 TT2] * [TT2 TT1 TT1], [TT1*TT2 TT1*TT1 TT2*TT1]);
    
    % scalar x vector
    tc.verifyEqual(TT1 * [TT2 TT1], [TT1*TT2 TT1*TT1]);
    
    % vector x scalar
    tc.verifyEqual([TT1 TT2]*TT2, [TT1*TT2 TT2*TT2]);
    
    %% SE3-vector product
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';

    % scalar x scalar
    
    tc.verifyEqual(TT1*vy, h2e( T1*e2h(vy) ), 'AbsTol', 1e-10);
    
    % vector x vector
    tc.verifyEqual([TT1 TT2 TT1] * [vz vx vy], [h2e(T1*e2h(vz)) h2e(T2*e2h(vx)) h2e(T1*e2h(vy))], 'AbsTol', 1e-10);
    
    % scalar x vector
    tc.verifyEqual(TT1 * [vx vy vz], h2e( T1*e2h([vx vy vz]) ), 'AbsTol', 1e-10);
    
    % vector x scalar
    tc.verifyEqual([TT1 TT2 TT1] * vy, [h2e(T1*e2h(vy)) h2e(T2*e2h(vy)) h2e(T1*e2h(vy))], 'AbsTol', 1e-10);
    
end

function function_tests(tc)
    
    % log
    T = SE2.exp([2 3 0.5]);
    tc.verifyEqual(log(T), [0 -0.5 2; 0.5 0 3; 0 0 0], 'AbsTol', 1e-10  );
    
end

function conversions_test(tc)
    
    
    %%  SE2                     convert to SE2 class

    TT = SE2(1, 2, 0.3);
    
    verifyClass(tc, TT.SE3, 'SE3');
    tc.verifyEqual(double(TT.SE3), transl(1, 2, 0) * trotz(0.3), 'AbsTol', 1e-10 );
    
    %% xyt
    tc.verifyEqual(TT.xyt(), [1 2, 0.3]', 'AbsTol', 1e-10);
    
    %% Twist
    T = SE2.exp([2 3 0.5]);
    t = T.Twist();
    verifyInstanceOf(tc, t, 'Twist');
    tc.verifyEqual(t.v, [2 3]', 'AbsTol', 1e-10);
    tc.verifyEqual(t.w, 0.5, 'AbsTol', 1e-10);
    
    %% Lie stuff
    th = 0.3; 
    RR = SO2(th);
    tc.verifyEqual(RR.log, skew(th), 'AbsTol', 1e-10 );

end

function adjoint_test(tc)
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    
    tc.verifyEqual(TT.Ad, [R skew(t)*R; zeros(3,3) R]);
    
    % velxform
    tc.verifyEqual(TT.velxform, [R zeros(3,3); zeros(3,3) R]);

end

function interp_test(tc)
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    I = SE3();
    
    tc.verifyEqual(double(interp(I, TT, 0)),   double(I), 'AbsTol', 1e-10 );
    tc.verifyEqual(double(interp(I, TT, 1)),   double(TT), 'AbsTol', 1e-4 );
    tc.verifyEqual(double(interp(I, TT, 0.5)), double(trinterp(TT, 0.5)), 'AbsTol', 1e-10  );
    
end



function miscellany_test(tc)
    
    TT = SE2(1, 2, 0.3);
    
    tc.verifyEqual(dim(TT), 3);
        
    verifyEqual( tc, isSE(TT), true );
    
    verifyClass(tc, TT.new, 'SE2');

    verifyClass(tc, SE2.check(TT), 'SE2');
    verifyClass(tc, SE2.check(TT.T), 'SE2');
    z = SE2.check(TT);
    tc.verifyEqual(double(z), double(TT));
    
    z = SE2.check(TT.T);
    tc.verifyEqual(double(z), TT.T);
    
end


function display_test(tc)
    
    T1 = SE3.rand;
    T2 = SE3.rand
    
    T1.print
    trprint(T1)   % old style syntax
    
    T1.plot
    
    T1.print
    trprint(T1)   % old style syntax
    
    T1.plot
    trplot(T1)   % old style syntax
    
    T1.animate
    T1.animate(T2)
    tranimate(T1)   % old style syntax
    tranimate(T1, T2)   % old style syntax
    tranimate2(T1)   % old style syntax
    tranimate2(T1, T2)   % old style syntax
end

