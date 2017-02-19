function tests = SE2Test
    tests = functiontests(localfunctions);
end


% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    verifyClass(tc, SE2(), 'SE2');

    %% null
    verifyEqual(tc, SE2().double, eye(3,3));
    
    %% translation only
    t = [1 2];
    verifyEqual(tc, SE2(t).double, transl2(t));
    verifyEqual(tc, SE2(t').double, transl2(t));
    verifyEqual(tc, SE2(t(1), t(2)).double, transl2(t));
    
    %% R
    R = rot2(-pi/2);
    verifyEqual(tc, SE2(R).double, r2t(R));
    
    %% R,t

    verifyEqual(tc, SE2(R,t).double, transl2(t)*r2t(R));
    
    %% T
    T = transl2(1, 2)*trot2(0.3);
    verifyEqual(tc, SE2(T).double, T);
    
    %% x,y,theta
    T = transl2(1, 2)*trot2(0.3);
    verifyEqual(tc, SE2(1, 2, 0.3).double, T);
    verifyEqual(tc, SE2([1, 2, 0.3]).double, T);
    verifyEqual(tc, SE2([1, 2], 0.3).double, T);
        
    %% T
    T = rt2tr(R,t);
    verifyEqual(tc, SE2(T).double, T);
    
    %% copy constructor
    TT = SE2(T);
    verifyEqual(tc, SE2(TT).double, T);
    
    
    %% vectorised versions
    
    T1 = transl2(1,2) * trot2(0.3);
    T2 = transl2(1,-2) * trot2(-0.4);
    
    TT = cat(3, T1, T2, T1, T2);

    tt = SE2(TT);
    verifyEqual(tc, length(tt), size(TT, 3) );
    verifyEqual(tc, tt.T, TT);
    
end

function staticconstructors_test(tc)
    
    %% exponential
    verifyEqual(tc, SE2.exp( skew(0.3) ).R, rot2(0.3), 'AbsTol', 1e-10  );
    
        
    %% exponential
    verifyEqual(tc, SE3.exp(zeros(4,4)).T, eye(4,4), 'AbsTol', 1e-10  );
    t= [1 2 3];
    verifyEqual(tc, SE3.exp([t 0 0 0]').T, transl(t), 'AbsTol', 1e-10  );
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
    verifyEqual(tc, double(TT1.inv()), inv(T1), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, double(TT1*TT1.inv()), eye(3,3), 'AbsTol', 1e-10  );
    verifyEqual(tc, double(TT1.inv()*TT1), eye(3,3), 'AbsTol', 1e-10  );
    
    % vector case
    verifyEqual(tc, double(TT1.inv()*TT1), eye(3,3), 'AbsTol', 1e-10  );
end

% function uminus_test(tc)
%     R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
%     
%     TT1 = SE3(T1);
% 
%     
%     TM = - TT1;
%     verifyEqual(tc, TM.double, -T1, 'AbsTol', 1e-10  );
% end

function Rt_test(tc)
    
    R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
    
    TT1 = SE3(T1);
    
    verifyEqual(tc, TT1.T, T1, 'AbsTol', 1e-10  );
    verifyEqual(tc, TT1.R, R1, 'AbsTol', 1e-10  );
    verifyEqual(tc, TT1.t, t1, 'AbsTol', 1e-10  );
    
    verifyEqual(tc, TT1.transl, t1', 'AbsTol', 1e-10  );
    TT = [TT1 TT1 TT1];
    verifyEqual(tc, TT.transl, [t1 t1 t1]', 'AbsTol', 1e-10  );
end


function arith_test(tc)
    
    R1 = rpy2r( randn(1,3) );  t1 = randn(3,1); T1 = rt2tr(R1, t1);
    R2 = rpy2r( randn(1,3) );  t2 = randn(3,1); T2 = rt2tr(R2, t2);
    
    TT1 = SE3(T1);
    TT2 = SE3(T2);

    I = SE3();
    
    
    %% SE3-SE3 product
    % scalar x scalar
    
    verifyEqual(tc, double(TT1*TT2), T1*T2, 'AbsTol', 1e-10  );
    verifyEqual(tc, double(TT2*TT1), T2*T1, 'AbsTol', 1e-10  );
    verifyEqual(tc, double(TT1*I), T1, 'AbsTol', 1e-10  );
    verifyEqual(tc, double(TT2*I), T2, 'AbsTol', 1e-10  );
    
    % vector x vector
    verifyEqual(tc, [TT1 TT1 TT2] * [TT2 TT1 TT1], [TT1*TT2 TT1*TT1 TT2*TT1]);
    
    % scalar x vector
    verifyEqual(tc, TT1 * [TT2 TT1], [TT1*TT2 TT1*TT1]);
    
    % vector x scalar
    verifyEqual(tc, [TT1 TT2]*TT2, [TT1*TT2 TT2*TT2]);
    
    %% SE3-vector product
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';

    % scalar x scalar
    
    verifyEqual(tc, TT1*vy, h2e( T1*e2h(vy) ), 'AbsTol', 1e-10);
    
    % vector x vector
    verifyEqual(tc, [TT1 TT2 TT1] * [vz vx vy], [h2e(T1*e2h(vz)) h2e(T2*e2h(vx)) h2e(T1*e2h(vy))], 'AbsTol', 1e-10);
    
    % scalar x vector
    verifyEqual(tc, TT1 * [vx vy vz], h2e( T1*e2h([vx vy vz]) ), 'AbsTol', 1e-10);
    
    % vector x scalar
    verifyEqual(tc, [TT1 TT2 TT1] * vy, [h2e(T1*e2h(vy)) h2e(T2*e2h(vy)) h2e(T1*e2h(vy))], 'AbsTol', 1e-10);
    
end

function function_tests(tc)
    
    % log
    T = SE2.exp([2 3 0.5]);
    verifyEqual(tc, log(T), [0 -0.5 2; 0.5 0 3; 0 0 0], 'AbsTol', 1e-10  );
    
end

function conversions_test(tc)
    
    
    %%  SE2                     convert to SE2 class

    TT = SE2(1, 2, 0.3);
    
    verifyClass(tc, TT.SE3, 'SE3');
    verifyEqual(tc, double(TT.SE3), transl(1, 2, 0) * trotz(0.3), 'AbsTol', 1e-10 );
    
    %% xyt
    verifyEqual(tc, TT.xyt(), [1 2, 0.3]', 'AbsTol', 1e-10);
    
    %% Twist
    T = SE2.exp([2 3 0.5]);
    t = T.Twist();
    verifyInstanceOf(tc, t, 'Twist');
    verifyEqual(tc, t.v, [2 3]', 'AbsTol', 1e-10);
    verifyEqual(tc, t.w, 0.5, 'AbsTol', 1e-10);
    
    %% Lie stuff
    th = 0.3; 
    RR = SO2(th);
    verifyEqual(tc, RR.log, skew(th), 'AbsTol', 1e-10 );

end

function adjoint_test(tc)
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    
    verifyEqual(tc, TT.Ad, [R skew(t)*R; zeros(3,3) R]);
    
    % velxform
    verifyEqual(tc, TT.velxform, [R zeros(3,3); zeros(3,3) R]);

end

function interp_test(tc)
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    I = SE3();
    
    verifyEqual(tc, double(interp(I, TT, 0)),   double(I), 'AbsTol', 1e-10 );
    verifyEqual(tc, double(interp(I, TT, 1)),   double(TT), 'AbsTol', 1e-4 );
    verifyEqual(tc, double(interp(I, TT, 0.5)), double(trinterp(TT, 0.5)), 'AbsTol', 1e-10  );
    
end



function miscellany_test(tc)
    
    TT = SE2(1, 2, 0.3);
    
    verifyEqual(tc, dim(TT), 3);
        
    verifyEqual( tc, isSE(TT), true );
    
    verifyClass(tc, TT.new, 'SE2');

    verifyClass(tc, SE2.check(TT), 'SE2');
    verifyClass(tc, SE2.check(TT.T), 'SE2');
    z = SE2.check(TT);
    verifyEqual(tc, double(z), double(TT));
    
    z = SE2.check(TT.T);
    verifyEqual(tc, double(z), TT.T);
    
end


function display_test(tc)
    
    r = SE2( 1, 2, 0.3 );
    
    r.print
    trprint(r)   % old style syntax
    
    r.plot
    
    %r.animate  no 2D animation method
end

