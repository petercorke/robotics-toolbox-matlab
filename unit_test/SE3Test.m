function tests = SE3Test
    tests = functiontests(localfunctions);
    clc
end


% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    verifyClass(tc, SE3(), 'SE3');

    
    %% null
    tc.verifyEqual(SE3().double, eye(4,4));
    
    %% translation only
    t = [1 2 3];
    verifyEqual(tc, SE3(t).double, transl(t));
    verifyEqual(tc, SE3(t').double, transl(t));
    verifyEqual(tc, SE3(t(1), t(2), t(3)).double, transl(t));
    

    
    %% R,t
    R = roty(-pi/2);
    verifyEqual(tc, SE3(R,t).double, transl(t)*r2t(R));
    
    %% T
    T = rt2tr(R,t);
    verifyEqual(tc, SE3(T).double, T);
    
    %% copy constructor
    TT = SE3(T);
    verifyEqual(tc, SE3(TT).double, T);
    
    
    %% vectorised versions
    
    T1 = transl(1,2,3) * rpy2tr(0.2, 0.3, 0.4);
    T2 = transl(1,-2,3) * rpy2tr(0.2, -0.3, 0.4);
    
    TT = cat(3, T1, T2, T1, T2);

    tt = SE3(TT);
    verifyEqual(tc, length(tt), size(TT, 3) );
    verifyEqual(tc, tt.T, TT);
    
    t = rand(10,3);
    s = SE3(t);
    tc.verifyLength(s, 10);
    tc.verifyEqual(s(4).t, t(4,:)');
    
    %% symbolic version
    syms x y z
    
        verifyClass(tc, SE3(x, y, z), 'SE3');

end

function double_test(tc)
    
    a = SE3(1,2, 3);
    d = double(a);
    verifyInstanceOf(tc, d, 'double');
    verifySize(tc, d, [4 4]);
    verifyEqual(tc, d(1:3,4), [1 2 3]');
    
    aa = [a a a a a];
    d = double(aa);
    verifyInstanceOf(tc, d, 'double');
    verifySize(tc, d, [4 4 5]);
    verifyEqual(tc, d(1:3,4, 1), [1 2 3]');
    verifyEqual(tc, d(1:3,4, 5), [1 2 3]');
end

function staticconstructors_test(tc)
    %% rotation primitives
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SE3.Rx(theta).T, trotx(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SE3.Ry(theta).T, troty(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SE3.Rz(theta).T, trotz(theta), 'AbsTol', 1e-10  );
    end
    
    for theta = [-pi/2 0 pi/2 pi]*180/pi
        verifyEqual(tc, SE3.Rx(theta, 'deg').T, trotx(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SE3.Ry(theta, 'deg').T, troty(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SE3.Rz(theta, 'deg').T, trotz(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    
    %% 3 angle
    verifyEqual(tc, SE3.rpy( 0.1, 0.2, 0.3 ).T, rpy2tr( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.rpy([ 0.1, 0.2, 0.3] ).T, rpy2tr( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.eul( 0.1, 0.2, 0.3 ).T, eul2tr( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.eul([ 0.1, 0.2, 0.3] ).T, eul2tr( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.rpy( 10, 20, 30, 'deg' ).T, rpy2tr( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.rpy([ 10, 20, 30], 'deg' ).T, rpy2tr( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.eul( 10, 20, 30, 'deg' ).T, eul2tr( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.eul([ 10, 20, 30], 'deg' ).T, eul2tr( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    %% OA vectors
    verifyEqual(tc, SE3.oa([0 1 0], [0 0 1]).T, eye(4,4), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.oa([1 0 0], [0 1 0]).T, [0 0 1 0; 1 0 0 0; 0 1 0 0; 0 0 0 1]', 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    verifyEqual(tc, SE3.angvec(th, v ).T, angvec2tr(th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.angvec(-th, v ).T, angvec2tr(-th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.angvec(-th, -v ).T, angvec2tr(-th, -v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SE3.angvec(th, -v ).T, angvec2tr(th, -v), 'AbsTol', 1e-10  );
    
    %% exponential
    verifyEqual(tc, SE3.exp(zeros(4,4)).T, eye(4,4), 'AbsTol', 1e-10  );
    t= [1 2 3];
    verifyEqual(tc, SE3.exp([t 0 0 0]').T, transl(t), 'AbsTol', 1e-10  );
end

function delta_test(tc)
    T = SE3(1,2,3) * SE3.Ry(pi/2);
    
    d = [4 5 6 1 2 3]*1e-3;
    
    % delta
    Td = SE3.delta(d);
    verifyEqual(tc, Td.t, d(1:3)', 'AbsTol', 1e-10);
    verifyEqual(tc, Td.R, skew(d(4:6))+eye(3,3), 'AbsTol', 1e-10);
    
    % increment
    T2 = T.increment(d)
    verifyEqual(tc, double(T2), double(T.*Td), 'AbsTol', 1e-10 );
end

function isa_test(tc)
    
    verifyTrue(tc, SE3.isa(trotx(0)) );
    verifyTrue(tc, SE3.isa(trotx(0), 'valid') );
    
    verifyFalse(tc, SE3.isa(1) )
end

function resulttype_test(tc)
    
    t = SE3();
    verifyClass(tc, t, 'SE3');
    
    verifyClass(tc, t*t, 'SE3');
    
    verifyClass(tc, t.*t, 'SE3');
    % other combos all fail, test this?

    verifyClass(tc, t/t, 'SE3');
    
    verifyClass(tc, inv(t), 'SE3');
    end

function inverse_test(tc)
    
    R1 = rpy2r( randn(1,3) );   t1 = randn(3,1);
    T1 = rt2tr(R1, t1); 
    
    TT1 = SE3(T1);
    
    % test inverse
    verifyEqual(tc, double(TT1.inv()), inv(T1), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, double(TT1*TT1.inv()), eye(4,4), 'AbsTol', 1e-10  );
    verifyEqual(tc, double(TT1.inv()*TT1), eye(4,4), 'AbsTol', 1e-10  );
    
    % vector case
    verifyEqual(tc, double(TT1.inv()*TT1), eye(4,4), 'AbsTol', 1e-10  );
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
    
    [a,b,c] = transl(TT1);
    verifyEqual(tc, a, t1(1));
    verifyEqual(tc, b, t1(2));
    verifyEqual(tc, c, t1(3));    
end

function tv_test(tc)
    a = SE3(1,2,3);
    verifyEqual(tc, a.tv, [1 2 3]');
    
    aa = [a a a a];
    verifyEqual(tc, aa.tv, [1 2 3]'*[1 1 1 1]);
    
end

function ctraj_test(tc)
    
    TT = SE3([2 4 6])*SE3.Rx(pi);
    I = SE3();
    
    tg = ctraj(I, TT, 3);
    verifyInstanceOf(tc, tg, 'SE3');
    verifySize(tc, tg, [1 3]);
    verifyEqual(tc, double(tg(1)), double(I), 'AbsTol', 1e-10 );
    verifyEqual(tc, double(tg(2)), double( SE3([2 4 6]/2)*SE3.Rx(pi/2) ), 'AbsTol', 1e-10 );
    verifyEqual(tc, double(tg(3)), double(TT), 'AbsTol', 1e-10 );
    
    tg = ctraj(I, TT, [1 0.5 0]);
    verifyInstanceOf(tc, tg, 'SE3');
    verifySize(tc, tg, [1 3]);
    verifyEqual(tc, double(tg(3)), double(I), 'AbsTol', 1e-10 );
    verifyEqual(tc, double(tg(2)), double( SE3([2 4 6]/2)*SE3.Rx(pi/2) ), 'AbsTol', 1e-10 );
    verifyEqual(tc, double(tg(1)), double(TT), 'AbsTol', 1e-10 );
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
    verifyEqual(tc, double(interp(I, TT, 0.5)), double(trinterp(T, 0.5)), 'AbsTol', 1e-10  );
    
end

function conversions_test(tc)
    
    %% 3 angle
    verifyEqual(tc, SE3.rpy( 0.1, 0.2, 0.3 ).torpy, [ 0.1, 0.2, 0.3], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.eul( 0.1, 0.2, 0.3 ).toeul, [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SE3.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    a = SE3.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    
    [a,b] = SE3.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    verifyEqual(tc, b, v, 'AbsTol', 1e-10  );
    
    %% quaternion
    
    R = rpy2r( 0.2, 0.3, 0.4);
    q = UnitQuaternion(R);
    T = SO3(R).SE3;
    verifyClass(tc, T.UnitQuaternion, 'UnitQuaternion');

    verifyEqual(tc, q, T.UnitQuaternion);
    
    %% Twist
    T = SE3.exp([1 2 3 0.2 0.3 0.4]);
    t = T.Twist();
    verifyInstanceOf(tc, t, 'Twist');
    verifyEqual(tc, t.v, [1 2 3]', 'AbsTol', 1e-10);
    verifyEqual(tc, t.w, [0.2 0.3 0.4], 'AbsTol', 1e-10);
    
    %%  SE3                     convert to SE3 class

    T = rpy2tr(0.2, 0.3, 0.4);
    verifyEqual(tc, SO3(T).SE3, SE3(T), 'AbsTol', 1e-10 );
    verifyEqual(tc, SO3(T).T, T, 'AbsTol', 1e-10 );

end


function miscellany_test(tc)
    
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    
    verifyEqual(tc, dim(TT), 4);
        
    verifyEqual( tc, isSE(TT), true );
    
    verifyClass(tc, TT.new, 'SE3');

    verifyClass(tc, SE3.check(TT), 'SE3');
    verifyClass(tc, SE3.check(T), 'SE3');
    z = SE3.check(TT);
    verifyEqual(tc, double(z), double(TT));
    
    z = SE3.check(T);
    verifyEqual(tc, double(z), T);
    
end

function display_test(tc)
    
    R = rpy2r( randn(1,3) );  t = randn(3,1); T = rt2tr(R, t);
    TT = SE3(T);
    
    TT.print
    trprint(TT)   % old style syntax
    
    TT.plot
    
    TT.animate
end

