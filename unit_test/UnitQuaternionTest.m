function tests = UnitQuaternionTest
  tests = functiontests(localfunctions);
end

%TODO
%    test new method
% test SO3 with symbolics

% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    verifyEqual(tc, UnitQuaternion().double, [1 0 0 0]);
    

    
    %% from S
    verifyEqual(tc, UnitQuaternion([1 0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, UnitQuaternion([0 1 0 0]).double, [0 1 0 0]);
    verifyEqual(tc, UnitQuaternion([0 0 1 0]).double, [0 0 1 0]);
    verifyEqual(tc, UnitQuaternion([0 0 0 1]).double, [0 0 0 1]);
 
    verifyEqual(tc, UnitQuaternion([2 0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, UnitQuaternion([-2 0 0 0]).double, [-1 0 0 0]);

    %% from [S,V]
    verifyEqual(tc, UnitQuaternion(1, [0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, UnitQuaternion(0, [1 0 0]).double, [0 1 0 0]);
    verifyEqual(tc, UnitQuaternion(0, [0 1 0]).double, [0 0 1 0]);
    verifyEqual(tc, UnitQuaternion(0, [0 0 1]).double, [0 0 0 1]);
 
    verifyEqual(tc, UnitQuaternion(2, [0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, UnitQuaternion(-2, [0 0 0]).double, [-1 0 0 0]);
    
    %% from R
    
    verifyEqual(tc, UnitQuaternion( eye(3,3) ).double, [1 0 0 0], 'AbsTol', 1e-10  );

    verifyEqual(tc, UnitQuaternion( rotx(pi/2) ).double, [1 1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( roty(pi/2) ).double, [1 0 1 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( rotz(pi/2) ).double, [1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion( rotx(-pi/2) ).double, [1 -1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( roty(-pi/2) ).double, [1 0 -1 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( rotz(-pi/2) ).double, [1 0 0 -1]/sqrt(2), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion( rotx(pi) ).double, [0 1 0 0], 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( roty(pi) ).double, [0 0 1 0], 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( rotz(pi) ).double, [0 0 0 1], 'AbsTol', 1e-10  );

    %% from T
    verifyEqual(tc, UnitQuaternion( trotx(pi/2) ).double, [1 1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( troty(pi/2) ).double, [1 0 1 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( trotz(pi/2) ).double, [1 0 0 1]/sqrt(2), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion( trotx(-pi/2) ).double, [1 -1 0 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( troty(-pi/2) ).double, [1 0 -1 0]/sqrt(2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( trotz(-pi/2) ).double, [1 0 0 -1]/sqrt(2), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion( trotx(pi) ).double, [0 1 0 0], 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( troty(pi) ).double, [0 0 1 0], 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( trotz(pi) ).double, [0 0 0 1], 'AbsTol', 1e-10  );
    
    %% vectorised forms of R, T
    R = []; T = [];
    for theta = [-pi/2 0 pi/2 pi]
        R = cat(3, R, rotx(theta), roty(theta), rotz(theta));
        T = cat(3, T, trotx(theta), troty(theta), trotz(theta));

    end
    verifyEqual(tc, UnitQuaternion(R).R, R, 'AbsTol', 1e-10);
    verifyEqual(tc, UnitQuaternion(T).T, T, 'AbsTol', 1e-10);
    
    %% copy constructor
    q = UnitQuaternion(rotx(0.3));
    verifyEqual(tc, UnitQuaternion(q), q, 'AbsTol', 1e-10);
    

end

function primitive_convert_test(tc)
    % char
    
    s = char( UnitQuaternion() );
    
    %% s,v
    verifyEqual(tc, UnitQuaternion([1 0 0 0]).s, 1);
    verifyEqual(tc, UnitQuaternion([1 0 0 0]).v, [0 0 0]);
    
    verifyEqual(tc, UnitQuaternion([0 1 0 0]).s, 0);
    verifyEqual(tc, UnitQuaternion([0 1 0 0]).v, [1 0 0]);
    
    verifyEqual(tc, UnitQuaternion([0 0 1 0]).s, 0);
    verifyEqual(tc, UnitQuaternion([0 0 1 0]).v, [0 1 0]);
    
    verifyEqual(tc, UnitQuaternion([0 0 0 1]).s, 0);
    verifyEqual(tc, UnitQuaternion([0 0 0 1]).v, [0 0 1]);

    
    %% R,T
     verifyEqual(tc, UnitQuaternion( eye(3,3) ).R, eye(3,3), 'AbsTol', 1e-10  );
   
    verifyEqual(tc, UnitQuaternion( rotx(pi/2) ).R, rotx(pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( roty(-pi/2) ).R, roty(-pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( rotz(pi) ).R, rotz(pi), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion( rotx(pi/2) ).T, trotx(pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( roty(-pi/2) ).T, troty(-pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion( rotz(pi) ).T, trotz(pi), 'AbsTol', 1e-10  );
end

function staticconstructors_test(tc)
    %% rotation primitives
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, UnitQuaternion.Rx(theta).R, rotx(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, UnitQuaternion.Ry(theta).R, roty(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, UnitQuaternion.Rz(theta).R, rotz(theta), 'AbsTol', 1e-10  );
    end
    
        for theta = [-pi/2 0 pi/2 pi]*180/pi
        verifyEqual(tc, UnitQuaternion.Rx(theta, 'deg').R, rotx(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, UnitQuaternion.Ry(theta, 'deg').R, roty(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, UnitQuaternion.Rz(theta, 'deg').R, rotz(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    
    %% 3 angle
    verifyEqual(tc, UnitQuaternion.rpy( 0.1, 0.2, 0.3 ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.rpy([ 0.1, 0.2, 0.3] ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.eul( 0.1, 0.2, 0.3 ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.eul([ 0.1, 0.2, 0.3] ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.rpy([ 10, 20, 30], 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.eul([ 10, 20, 30], 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    verifyEqual(tc, UnitQuaternion.angvec(th, v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.angvec(-th, v ).R, angvec2r(-th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.angvec(-th, -v ).R, angvec2r(-th, -v), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.angvec(th, -v ).R, angvec2r(th, -v), 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    verifyEqual(tc, UnitQuaternion.omega(th*v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, UnitQuaternion.omega(-th*v ).R, angvec2r(-th, v), 'AbsTol', 1e-10  );
    
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
    
    verifyEqual(tc, rx*u, rx);
    verifyEqual(tc, u*rx, rx); 
    
    % vector x vector
    verifyEqual(tc, [ry rz rx] * [rx ry rz], [ry*rx rz*ry rx*rz]);
    
    % scalar x vector
    verifyEqual(tc, ry * [rx ry rz], [ry*rx ry*ry ry*rz]);
    
    % vector x scalar
    verifyEqual(tc, [rx ry rz] * ry, [rx*ry ry*ry rz*ry]);
    
    %% quat-vector product
    % scalar x scalar
    
    verifyEqual(tc, rx*vy, vz, 'AbsTol', 1e-10);
    
    % vector x vector
    verifyEqual(tc, [ry rz rx] * [vz vx vy], [vx vy vz], 'AbsTol', 1e-10);
    
    % scalar x vector
    verifyEqual(tc, ry * [vx vy vz], [-vz vy vx], 'AbsTol', 1e-10);
    
    % vector x scalar
    verifyEqual(tc, [ry rz rx] * vy, [vy -vx vz], 'AbsTol', 1e-10);
end

function multiply_normalized_test(tc)
    
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    %% quat-quat product
    % scalar x scalar
    
    verifyEqual(tc, double(rx.*u), double(rx), 'AbsTol', 1e-10);
    verifyEqual(tc, double(u.*rx), double(rx), 'AbsTol', 1e-10);
    
    % shouldn't make that much difference here
    verifyEqual(tc, double(rx.*ry), double(rx*ry), 'AbsTol', 1e-10);
    verifyEqual(tc, double(rx.*rz), double(rx*rz), 'AbsTol', 1e-10); 
    
    % vector x vector
    verifyEqual(tc, [ry rz rx] .* [rx ry rz], [ry.*rx rz.*ry rx.*rz], 'AbsTol', 1e-10);
    
    % scalar x vector
    verifyEqual(tc, ry .* [rx ry rz], [ry.*rx ry.*ry ry.*rz], 'AbsTol', 1e-10);
    
    % vector x scalar
    verifyEqual(tc, [rx ry rz] .* ry, [rx.*ry ry.*ry rz.*ry], 'AbsTol', 1e-10);
    
end

function divide_test(tc)
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    % scalar / scalar
    % implicity tests inv

    verifyEqual(tc, rx/u, rx);
    verifyEqual(tc, ry/ry, u);

    % vector / vector
    verifyEqual(tc, [ry rz rx] / [rx ry rz], [ry/rx rz/ry rx/rz]);
    
    % vector / scalar
    verifyEqual(tc, [rx ry rz] / ry, [rx/ry ry/ry rz/ry]);
end

function divide_normalized_test(tc)
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    % scalar / scalar
    
    % shouldn't make that much difference here
    verifyEqual(tc, double(rx./ry), double(rx/ry), 'AbsTol', 1e-10);
    verifyEqual(tc, double(rx./rz), double(rx/rz), 'AbsTol', 1e-10); 
    
    verifyEqual(tc, double(rx./u), double(rx), 'AbsTol', 1e-10);
    verifyEqual(tc, double(ry./ry), double(u), 'AbsTol', 1e-10);

    % vector / vector
    verifyEqual(tc, [ry rz rx] ./ [rx ry rz], [ry./rx rz./ry rx./rz], 'AbsTol', 1e-10);
    
    % vector / scalar
    verifyEqual(tc, [rx ry rz] ./ ry, [rx./ry ry./ry rz./ry], 'AbsTol', 1e-10);
end

function angle_test(tc)
        %% angle between quaternions
    %% pure
    v = [5 6 7];
end

function conversions_test(tc)
    
    %% 3 angle
    verifyEqual(tc, UnitQuaternion.rpy( 0.1, 0.2, 0.3 ).torpy, [ 0.1, 0.2, 0.3], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.eul( 0.1, 0.2, 0.3 ).toeul, [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, UnitQuaternion.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    a = UnitQuaternion.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    
    [a,b] = UnitQuaternion.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    verifyEqual(tc, b, v, 'AbsTol', 1e-10  );
    

%  SO3                     convert to SO3 class
%  SE3                     convert to SE3 class
end

function miscellany_test(tc)
    
    rx = UnitQuaternion.Rx(pi/2);
    ry = UnitQuaternion.Ry(pi/2);
    rz = UnitQuaternion.Rz(pi/2);
    u = UnitQuaternion();
    
    %% norm
    verifyEqual(tc, rx.norm, 1, 'AbsTol', 1e-10  );
    verifyEqual(tc, norm([rx ry rz]), [1 1 1]', 'AbsTol', 1e-10  );
    
    %% unit
    verifyEqual(tc, rx.unit, rx, 'AbsTol', 1e-10  );
    verifyEqual(tc, unit([rx ry rz]), [rx ry rz], 'AbsTol', 1e-10  );
    
    %% inner
    verifyEqual(tc, u.inner(u), 1, 'AbsTol', 1e-10  );
    verifyEqual(tc, rx.inner(ry), 0.5, 'AbsTol', 1e-10  );
    verifyEqual(tc, rz.inner(rz), 1, 'AbsTol', 1e-10  );

    %% interp
    q = rx*ry*rz;
    verifyEqual(tc, q.interp(0), u, 'AbsTol', 1e-10  );
    verifyEqual(tc, q.interp(1), q, 'AbsTol', 1e-10  );
    verifyEqual(tc, q.interp(rx, 0), q, 'AbsTol', 1e-10  );
    verifyEqual(tc, q.interp(rx, 1), rx, 'AbsTol', 1e-10  );
    
    verifyEqual(tc, q^0, u, 'AbsTol', 1e-10  );
    verifyEqual(tc, q^(-1), inv(q), 'AbsTol', 1e-10  );
    verifyEqual(tc, q^2, q*q, 'AbsTol', 1e-10  );
    
    %% equality tests
    verifyTrue(tc, rx == rx);
    verifyFalse(tc, rx ~= rx);
    verifyFalse(tc, rx == ry);
    


end

function dot_test(tc)
    q = UnitQuaternion();
    omega = [1 2 3];
    
    verifyEqual(tc, q.dot(omega), [0 omega/2]');
    verifyEqual(tc, q.dotb(omega), [0 omega/2]');
    
    q = UnitQuaternion(rotx(pi/2));
    verifyEqual(tc, q.dot(omega), double(0.5*Quaternion.pure(omega)*q)' );
    verifyEqual(tc, q.dotb(omega), double(0.5*q*Quaternion.pure(omega))' );

end

function display_test(tc)
        ry = UnitQuaternion.Ry(pi/2);

        
        ry.plot
        ry.animate
end