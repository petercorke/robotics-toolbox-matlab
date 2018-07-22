function tests = UnitQuaternionTest
  tests = functiontests(localfunctions);
  clc
end

%TODO
%    test new method
% test SO3 with symbolics

% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    tc.verifyEqual(UnitQuaternion().double, [1 0 0 0]);
    

    
    %% from S
    tc.verifyEqual(UnitQuaternion([1 0 0 0]).double, [1 0 0 0]);
    tc.verifyEqual(UnitQuaternion([0 1 0 0]).double, [0 1 0 0]);
    tc.verifyEqual(UnitQuaternion([0 0 1 0]).double, [0 0 1 0]);
    tc.verifyEqual(UnitQuaternion([0 0 0 1]).double, [0 0 0 1]);
 
    tc.verifyEqual(UnitQuaternion([2 0 0 0]).double, [1 0 0 0]);
    tc.verifyEqual(UnitQuaternion([-2 0 0 0]).double, [-1 0 0 0]);

    %% from [S,V]
    tc.verifyEqual(UnitQuaternion(1, [0 0 0]).double, [1 0 0 0]);
    tc.verifyEqual(UnitQuaternion(0, [1 0 0]).double, [0 1 0 0]);
    tc.verifyEqual(UnitQuaternion(0, [0 1 0]).double, [0 0 1 0]);
    tc.verifyEqual(UnitQuaternion(0, [0 0 1]).double, [0 0 0 1]);
 
    tc.verifyEqual(UnitQuaternion(2, [0 0 0]).double, [1 0 0 0]);
    tc.verifyEqual(UnitQuaternion(-2, [0 0 0]).double, [-1 0 0 0]);
    
    %% from R
    
    tc.verifyEqual(UnitQuaternion( eye(3,3) ).double, [1 0 0 0], 'AbsTol', 1e-10  );

    tc.verifyEqual(UnitQuaternion( rotx(pi/2) ).double, [1 1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( roty(pi/2) ).double, [1 0 1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( rotz(pi/2) ).double, [1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( rotx(-pi/2) ).double, [1 -1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( roty(-pi/2) ).double, [1 0 -1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( rotz(-pi/2) ).double, [1 0 0 -1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( rotx(pi) ).double, [0 1 0 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( roty(pi) ).double, [0 0 1 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( rotz(pi) ).double, [0 0 0 1], 'AbsTol', 1e-10  );
    
    %% from SO3
    
    tc.verifyEqual(UnitQuaternion( SO3 ).double, [1 0 0 0], 'AbsTol', 1e-10  );

    tc.verifyEqual(UnitQuaternion( SO3.Rx(pi/2) ).double, [1 1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Ry(pi/2) ).double, [1 0 1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Rz(pi/2) ).double, [1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( SO3.Rx(-pi/2) ).double, [1 -1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Ry(-pi/2) ).double, [1 0 -1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Rz(-pi/2) ).double, [1 0 0 -1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( SO3.Rx(pi) ).double, [0 1 0 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Ry(pi) ).double, [0 0 1 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( SO3.Rz(pi) ).double, [0 0 0 1], 'AbsTol', 1e-10  );
    
    % vector of SO3
    tc.verifyEqual(UnitQuaternion( [SO3.Rx(pi/2) SO3.Ry(pi/2) SO3.Rz(pi/2)] ).double, [1 1 0 0; 1 0 1 0; 1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );

    %% from T
    tc.verifyEqual(UnitQuaternion( trotx(pi/2) ).double, [1 1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( troty(pi/2) ).double, [1 0 1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( trotz(pi/2) ).double, [1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( trotx(-pi/2) ).double, [1 -1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( troty(-pi/2) ).double, [1 0 -1 0]/sqrt(2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( trotz(-pi/2) ).double, [1 0 0 -1]/sqrt(2), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( trotx(pi) ).double, [0 1 0 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( troty(pi) ).double, [0 0 1 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( trotz(pi) ).double, [0 0 0 1], 'AbsTol', 1e-10  );
    
    %% vectorised forms of R, T
    R = []; T = [];
    for theta = [-pi/2 0 pi/2 pi]
        R = cat(3, R, rotx(theta), roty(theta), rotz(theta));
        T = cat(3, T, trotx(theta), troty(theta), trotz(theta));

    end
    tc.verifyEqual(UnitQuaternion(R).R, R, 'AbsTol', 1e-10);
    tc.verifyEqual(UnitQuaternion(T).T, T, 'AbsTol', 1e-10);
    
    %% copy constructor
    q = UnitQuaternion(rotx(0.3));
    tc.verifyEqual(UnitQuaternion(q), q, 'AbsTol', 1e-10);
    

end

function primitive_convert_test(tc)
    % char
    
    u = UnitQuaternion();
    
    s = char( u );
    s = char( [u u u] );
    u
    [u u]
    
    %% s,v
    tc.verifyEqual(UnitQuaternion([1 0 0 0]).s, 1);
    tc.verifyEqual(UnitQuaternion([1 0 0 0]).v, [0 0 0]);
    
    tc.verifyEqual(UnitQuaternion([0 1 0 0]).s, 0);
    tc.verifyEqual(UnitQuaternion([0 1 0 0]).v, [1 0 0]);
    
    tc.verifyEqual(UnitQuaternion([0 0 1 0]).s, 0);
    tc.verifyEqual(UnitQuaternion([0 0 1 0]).v, [0 1 0]);
    
    tc.verifyEqual(UnitQuaternion([0 0 0 1]).s, 0);
    tc.verifyEqual(UnitQuaternion([0 0 0 1]).v, [0 0 1]);

    
    %% R,T
    tc.verifyEqual(u.R, eye(3,3), 'AbsTol', 1e-10  );
   
    tc.verifyEqual(UnitQuaternion( rotx(pi/2) ).R, rotx(pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( roty(-pi/2) ).R, roty(-pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( rotz(pi) ).R, rotz(pi), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion( rotx(pi/2) ).T, trotx(pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( roty(-pi/2) ).T, troty(-pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion( rotz(pi) ).T, trotz(pi), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.q2r(u.double), eye(3,3), 'AbsTol', 1e-10  );

end

function staticconstructors_test(tc)
    %% rotation primitives
    for theta = [-pi/2 0 pi/2 pi]
        tc.verifyEqual(UnitQuaternion.Rx(theta).R, rotx(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        tc.verifyEqual(UnitQuaternion.Ry(theta).R, roty(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        tc.verifyEqual(UnitQuaternion.Rz(theta).R, rotz(theta), 'AbsTol', 1e-10  );
    end
    
        for theta = [-pi/2 0 pi/2 pi]*180/pi
        tc.verifyEqual(UnitQuaternion.Rx(theta, 'deg').R, rotx(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        tc.verifyEqual(UnitQuaternion.Ry(theta, 'deg').R, roty(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        tc.verifyEqual(UnitQuaternion.Rz(theta, 'deg').R, rotz(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    
    %% 3 angle
    tc.verifyEqual(UnitQuaternion.rpy( 0.1, 0.2, 0.3 ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.rpy([ 0.1, 0.2, 0.3] ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.eul( 0.1, 0.2, 0.3 ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.eul([ 0.1, 0.2, 0.3] ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.rpy([ 10, 20, 30], 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.eul([ 10, 20, 30], 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    tc.verifyEqual(UnitQuaternion.angvec(th, v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.angvec(-th, v ).R, angvec2r(-th, v), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.angvec(-th, -v ).R, angvec2r(-th, -v), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.angvec(th, -v ).R, angvec2r(th, -v), 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    tc.verifyEqual(UnitQuaternion.omega(th*v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    tc.verifyEqual(UnitQuaternion.omega(-th*v ).R, angvec2r(-th, v), 'AbsTol', 1e-10  );
    
end

function canonic_test(tc)
    R = rotx(0);
    tc.verifyEqual( UnitQuaternion(R).double, [1 0 0 0], 'AbsTol', 1e-12);
    
    R = rotx(pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[1 0 0]], 'AbsTol', 1e-12);
    R = roty(pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[0 1 0]], 'AbsTol', 1e-12);
    R = rotz(pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[0 0 1]], 'AbsTol', 1e-12);
    
    R = rotx(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[-1 0 0]], 'AbsTol', 1e-12);
    R = roty(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[0 -1 0]], 'AbsTol', 1e-12);
    R = rotz(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/4) sin(pi/4)*[0 0 -1]], 'AbsTol', 1e-12);
    
    R = rotx(pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[1 0 0]], 'AbsTol', 1e-12);
    R = roty(pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[0 1 0]], 'AbsTol', 1e-12);
    R = rotz(pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[0 0 1]], 'AbsTol', 1e-12);
    
    R = rotx(-pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[1 0 0]], 'AbsTol', 1e-12);
    R = roty(-pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[0 1 0]], 'AbsTol', 1e-12);
    R = rotz(-pi);
    tc.verifyEqual( UnitQuaternion(R).double, [cos(pi/2) sin(pi/2)*[0 0 1]], 'AbsTol', 1e-12);
end

function convert_test(tc)
    % test conversion from rotn matrix to u.quaternion and back
    R = rotx(0);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    
    R = rotx(pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = roty(pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = rotz(pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    
    R = rotx(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = roty(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = rotz(-pi/2);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    
    R = rotx(pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = roty(pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = rotz(pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    
    R = rotx(-pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = roty(-pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
    R = rotz(-pi);
    tc.verifyEqual( UnitQuaternion(R).R, R, 'AbsTol', 1e-12);
end

function resulttype_test(tc)
    
    q = Quaternion([2 0 0 0]);
    u = UnitQuaternion();
    
    verifyClass(tc, q*q, 'Quaternion');
    verifyClass(tc, q*u, 'Quaternion');
    verifyClass(tc, u*q, 'Quaternion');
    verifyClass(tc, u*u, 'UnitQuaternion');
    
    verifyClass(tc, u.*u, 'UnitQuaternion');
    % other combos all fail, test this?
    
    verifyClass(tc, q/q, 'Quaternion');
    verifyClass(tc, q/u, 'Quaternion');
    verifyClass(tc, u/u, 'UnitQuaternion');
    
    verifyClass(tc, conj(u), 'UnitQuaternion');
    verifyClass(tc, inv(u), 'UnitQuaternion');
    verifyClass(tc, unit(u), 'UnitQuaternion');
    verifyClass(tc, unit(q), 'UnitQuaternion');
    
    verifyClass(tc, conj(q), 'Quaternion');
    verifyClass(tc, inv(q), 'Quaternion');
    
    verifyClass(tc, q+q, 'Quaternion');
    verifyClass(tc, q-q, 'Quaternion');
    
    verifyClass(tc, u.SO3, 'SO3');
    verifyClass(tc, u.SE3, 'SE3');

end

function multiply_test(tc)
    
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    %% quat-quat product
    % scalar x scalar
    
    tc.verifyEqual(rx*u, rx);
    tc.verifyEqual(u*rx, rx); 
    
    % vector x vector
    tc.verifyEqual([ry rz rx] * [rx ry rz], [ry*rx rz*ry rx*rz]);
    
    % scalar x vector
    tc.verifyEqual(ry * [rx ry rz], [ry*rx ry*ry ry*rz]);
    
    % vector x scalar
    tc.verifyEqual([rx ry rz] * ry, [rx*ry ry*ry rz*ry]);
    
    %% quat-vector product
    % scalar x scalar
    
    tc.verifyEqual(rx*vy, vz, 'AbsTol', 1e-10);
    
    % vector x vector
    tc.verifyEqual([ry rz rx] * [vz vx vy], [vx vy vz], 'AbsTol', 1e-10);
    
    % scalar x vector
    tc.verifyEqual(ry * [vx vy vz], [-vz vy vx], 'AbsTol', 1e-10);
    
    % vector x scalar
    tc.verifyEqual([ry rz rx] * vy, [vy -vx vz], 'AbsTol', 1e-10);
end

function multiply_normalized_test(tc)
    
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    %% quat-quat product
    % scalar x scalar
    
    tc.verifyEqual(double(rx.*u), double(rx), 'AbsTol', 1e-10);
    tc.verifyEqual(double(u.*rx), double(rx), 'AbsTol', 1e-10);
    
    % shouldn't make that much difference here
    tc.verifyEqual(double(rx.*ry), double(rx*ry), 'AbsTol', 1e-10);
    tc.verifyEqual(double(rx.*rz), double(rx*rz), 'AbsTol', 1e-10); 
    
    % vector x vector
    tc.verifyEqual([ry rz rx] .* [rx ry rz], [ry.*rx rz.*ry rx.*rz], 'AbsTol', 1e-10);
    
    % scalar x vector
    tc.verifyEqual(ry .* [rx ry rz], [ry.*rx ry.*ry ry.*rz], 'AbsTol', 1e-10);
    
    % vector x scalar
    tc.verifyEqual([rx ry rz] .* ry, [rx.*ry ry.*ry rz.*ry], 'AbsTol', 1e-10);
    
end

function divide_test(tc)
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    % scalar / scalar
    % implicity tests inv

    tc.verifyEqual(rx/u, rx);
    tc.verifyEqual(ry/ry, u);

    % vector / vector
    tc.verifyEqual([ry rz rx] / [rx ry rz], [ry/rx rz/ry rx/rz]);
    
    % vector / scalar
    tc.verifyEqual([rx ry rz] / ry, [rx/ry ry/ry rz/ry]);
    
    % scalar / vector
    tc.verifyEqual(ry / [rx ry rz], [ry/rx ry/ry ry/rz]);
end

function divide_normalized_test(tc)
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    % scalar / scalar
    
    % shouldn't make that much difference here
    tc.verifyEqual(double(rx./ry), double(rx/ry), 'AbsTol', 1e-10);
    tc.verifyEqual(double(rx./rz), double(rx/rz), 'AbsTol', 1e-10); 
    
    tc.verifyEqual(double(rx./u), double(rx), 'AbsTol', 1e-10);
    tc.verifyEqual(double(ry./ry), double(u), 'AbsTol', 1e-10);

    % vector / vector
    tc.verifyEqual([ry rz rx] ./ [rx ry rz], [ry./rx rz./ry rx./rz], 'AbsTol', 1e-10);
    
    % vector / scalar
    tc.verifyEqual([rx ry rz] ./ ry, [rx./ry ry./ry rz./ry], 'AbsTol', 1e-10);
    
   % scalar / vector
    tc.verifyEqual(ry ./ [rx ry rz], [ry./rx ry./ry ry./rz]);
end

function angle_test(tc)
        %% angle between quaternions
    %% pure
    v = [5 6 7];
end

function conversions_test(tc)
    
    %% 3 angle
    tc.verifyEqual(UnitQuaternion.rpy( 0.1, 0.2, 0.3 ).torpy, [ 0.1, 0.2, 0.3], 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.eul( 0.1, 0.2, 0.3 ).toeul, [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    tc.verifyEqual(UnitQuaternion.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    a = UnitQuaternion.angvec(th, v ).toangvec;
    tc.verifyEqual(a, th, 'AbsTol', 1e-10  );
    
    [a,b] = UnitQuaternion.angvec(th, v ).toangvec;
    tc.verifyEqual(a, th, 'AbsTol', 1e-10  );
    tc.verifyEqual(b, v, 'AbsTol', 1e-10  );
    
    % null rotation case
    th = 0; v = unit([1 2 3]);
    a = UnitQuaternion.angvec(th, v ).toangvec;
    tc.verifyEqual(a, th, 'AbsTol', 1e-10  );
    

%  SO3                     convert to SO3 class
%  SE3                     convert to SE3 class
end

function miscellany_test(tc)
    
    % AbsTol not used since Quaternion supports eq() operator
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    %% norm
    tc.verifyEqual(rx.norm, 1, 'AbsTol', 1e-10  );
    tc.verifyEqual(norm([rx ry rz]), [1 1 1]', 'AbsTol', 1e-10  );
    
    %% unit
    tc.verifyEqual(rx.unit, rx, 'AbsTol', 1e-10  );
    tc.verifyEqual(unit([rx ry rz]), [rx ry rz], 'AbsTol', 1e-10  );
    
    %% inner
    tc.verifyEqual(u.inner(u), 1, 'AbsTol', 1e-10  );
    tc.verifyEqual(rx.inner(ry), 0.5, 'AbsTol', 1e-10  );
    tc.verifyEqual(rz.inner(rz), 1, 'AbsTol', 1e-10  );


    q = rx*ry*rz;
        
    tc.verifyEqual(q^0, u, 'AbsTol', 1e-10  );
    tc.verifyEqual(q^(-1), inv(q), 'AbsTol', 1e-10  );
    tc.verifyEqual(q^2, q*q, 'AbsTol', 1e-10  );
    
    %% angle
    tc.verifyEqual(angle(u, u), 0, 'AbsTol', 1e-10  );
    tc.verifyEqual(angle(u, rx), pi/4, 'AbsTol', 1e-10  );
    tc.verifyEqual(angle(u, [rx u]), pi/4*[1 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(angle([rx u], u), pi/4*[1 0], 'AbsTol', 1e-10  );
    tc.verifyEqual(angle([rx u], [u rx]), pi/4*[1 1], 'AbsTol', 1e-10  );

    
    %% increment
    w = [0.02 0.03 0.04];
    
    tc.verifyEqual(rx.increment(w), rx*UnitQuaternion.omega(w));

end

function interp_test(tc)
        
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    q = rx*ry*rz;
    
    % from null
    tc.verifyEqual(q.interp(0), u);
    tc.verifyEqual(q.interp(1), q );
    
    tc.verifyEqual(length(q.interp(linspace(0,1, 10))), 10);
    verifyTrue(tc, all( q.interp([0 1]) == [u q]));
    
    q0_5 = q.interp(0.5);
    tc.verifyEqual( q0_5 * q0_5, q);
    
    % between two quaternions
    tc.verifyEqual(q.interp(rx, 0), q );
    tc.verifyEqual(q.interp(rx, 1), rx );
    
    verifyTrue(tc, all( q.interp(rx, [0 1]) == [q rx]));
    
    % test shortest option
    q1 = UnitQuaternion.Rx(0.9*pi);
    q2 = UnitQuaternion.Rx(-0.9*pi);
    qq = q1.interp(q2, 11);
    tc.verifyEqual( qq(6), UnitQuaternion.Rx(0) )
    qq = q1.interp(q2, 11, 'shortest');
    tc.verifyEqual( qq(6), UnitQuaternion.Rx(pi) )
end

function eq_test(tc)
    q1 = UnitQuaternion([0 1 0 0]);
	q2 = UnitQuaternion([0 -1 0 0]);
    q3 = UnitQuaternion.Rz(pi/2);
    
    tc.verifyTrue( q1 == q1);
    tc.verifyTrue( q2 == q2);
    tc.verifyTrue( q3 == q3);
    tc.verifyTrue( q1 == q2);
    tc.verifyFalse( q1 == q3);
    
    tc.verifyEqual( [q1 q1 q1] == [q1 q1 q1], [true true true]);
    tc.verifyEqual( [q1 q2 q3] == [q1 q2 q3], [true true true]);
    tc.verifyEqual( [q1 q1 q3] == q1, [true true false]);
    tc.verifyEqual( q3 == [q1 q1 q3], [false false true]);
end


function logical_test(tc)
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    
    %% equality tests
    verifyTrue(tc, rx == rx);
    verifyFalse(tc, rx ~= rx);
    verifyFalse(tc, rx == ry);
end



function dot_test(tc)
    q = UnitQuaternion();
    omega = [1 2 3];
    
    tc.verifyEqual(q.dot(omega), [0 omega/2]');
    tc.verifyEqual(q.dotb(omega), [0 omega/2]');
    
    q = UnitQuaternion.Rx(pi/2);
    tc.verifyEqual(q.dot(omega), double(0.5*Quaternion.pure(omega)*q)', 'AbsTol', 1e-10 );
    tc.verifyEqual(q.dotb(omega), double(0.5*q*Quaternion.pure(omega))', 'AbsTol', 1e-10 );

end

function matrix_test(tc)
    
    q1 = UnitQuaternion.rpy(0.1, 0.2, 0.3);
    q2 = UnitQuaternion.rpy(0.2, 0.3, 0.4);
    
    q12 = q1 * q2;
    
    tc.verifyEqual(double(q12)', q1.matrix() * q2.double', 'AbsTol', 1e-10);
end

function vec3_test(tc)
    
    q1 = UnitQuaternion.rpy(0.1, 0.2, 0.3);
    q2 = UnitQuaternion.rpy(0.2, 0.3, 0.4);
    
    q12 = q1 * q2;
    
    q1v = q1.tovec; q2v = q2.tovec;
    
    q12v = UnitQuaternion.qvmul(q1v, q2v);
    
    q12_ = UnitQuaternion.vec(q12v);
    
    tc.verifyEqual(q12, q12_);
end

 
function display_test(tc)
        ry = UnitQuaternion.Ry(pi/2);

        ry.plot();
        h = ry.plot();
        ry.animate();
        ry.animate('rgb');
        ry.animate( UnitQuaternion.Rx(pi/2), 'rgb' )
end