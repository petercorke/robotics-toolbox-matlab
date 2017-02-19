function tests = SO3Test
  tests = functiontests(localfunctions);
end


% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    verifyEqual(tc, SO3().double, eye(3,3));
    
    
    %% from R
    
    verifyEqual(tc, SO3( eye(3,3) ).double, eye(3,3), 'AbsTol', 1e-10  );

    verifyEqual(tc, SO3( rotx(pi/2) ).double, rotx(pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( roty(-pi/2) ).double, roty(-pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( rotz(pi) ).double, rotz(pi), 'AbsTol', 1e-10  );

    %% from T
    verifyEqual(tc, SO3( trotx(pi/2) ).double, rotx(pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( troty(-pi/2) ).double, roty(-pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( trotz(pi) ).double, rotz(pi), 'AbsTol', 1e-10  );
    
    %% R,T
    verifyEqual(tc, SO3( eye(3,3) ).R, eye(3,3), 'AbsTol', 1e-10  );
   
    verifyEqual(tc, SO3( rotx(pi/2) ).R, rotx(pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( roty(-pi/2) ).R, roty(-pi/2), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3( rotz(pi) ).R, rotz(pi), 'AbsTol', 1e-10  );
    
    
    %% vectorised forms of R
    R = [];
    for theta = [-pi/2 0 pi/2 pi]
        R = cat(3, R, rotx(theta), roty(theta), rotz(theta));

    end
    verifyEqual(tc, SO3(R).R, R, 'AbsTol', 1e-10);
    
    %% copy constructor
    r = SO3(rotx(0.3));
    verifyEqual(tc, SO3(r), r, 'AbsTol', 1e-10);
    

end

function primitive_convert_test(tc)
    % char
    
    s = char( SO3() );
   
end

function staticconstructors_test(tc)
    %% rotation primitives
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SO3.Rx(theta).R, rotx(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SO3.Ry(theta).R, roty(theta), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SO3.Rz(theta).R, rotz(theta), 'AbsTol', 1e-10  );
    end
    
    for theta = [-pi/2 0 pi/2 pi]*180/pi
        verifyEqual(tc, SO3.Rx(theta, 'deg').R, rotx(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SO3.Ry(theta, 'deg').R, roty(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    for theta = [-pi/2 0 pi/2 pi]
        verifyEqual(tc, SO3.Rz(theta, 'deg').R, rotz(theta, 'deg'), 'AbsTol', 1e-10  );
    end
    
    %% 3 angle
    verifyEqual(tc, SO3.rpy( 0.1, 0.2, 0.3 ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.rpy([ 0.1, 0.2, 0.3] ).R, rpy2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.eul( 0.1, 0.2, 0.3 ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.eul([ 0.1, 0.2, 0.3] ).R, eul2r( 0.1, 0.2, 0.3 ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.rpy([ 10, 20, 30], 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.eul([ 10, 20, 30], 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );

    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    verifyEqual(tc, SO3.angvec(th, v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.angvec(-th, v ).R, angvec2r(-th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.angvec(-th, -v ).R, angvec2r(-th, -v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.angvec(th, -v ).R, angvec2r(th, -v), 'AbsTol', 1e-10  );
    
    %% exponential
    verifyEqual(tc, SO3.exp([0.3 0 0]).R, rotx(0.3), 'AbsTol', 1e-10  );

    th = 0.2; v = unit([1 2 3]);
    verifyEqual(tc, SO3.exp( th*v ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.exp( th*skew(v) ).R, angvec2r(th, v), 'AbsTol', 1e-10  );
    
    %% OA vectors
    verifyEqual(tc, SO3.oa([0 1 0], [0 0 1]).R, eye(3,3), 'AbsTol', 1e-10  );
    verifyEqual(tc, SO3.oa([1 0 0], [0 1 0]).R, [0 0 1; 1 0 0; 0 1 0]', 'AbsTol', 1e-10  );

end

function isa_test(tc)
    
    verifyTrue(tc, SO3.isa(rotx(0)) );
    verifyTrue(tc, SO3.isa(rotx(0)), 'valid' );
    
    verifyFalse(tc, SO3.isa(1) )
end

function resulttype_test(tc)
    
    r = SO3();
    verifyClass(tc, r, 'SO3');

    verifyClass(tc, r*r, 'SO3');
    
    verifyClass(tc, r.*r, 'SO3');
    % other combos all fail, test this?
    

    verifyClass(tc, r/r, 'SO3');
    
    verifyClass(tc, inv(r), 'SO3');
    
end

function multiply_test(tc)
    
    vx = [1 0 0]'; vy = [0 1 0]'; vz = [0 0 1]';
    rx = SO3.Rx(pi/2);
    ry = SO3.Ry(pi/2);
    rz = SO3.Rz(pi/2);
    u = SO3();
    
    %% SO3-SO3 product
    % scalar x scalar
    
    verifyEqual(tc, rx*u, rx);
    verifyEqual(tc, u*rx, rx); 
    
    % vector x vector
    verifyEqual(tc, [ry rz rx] * [rx ry rz], [ry*rx rz*ry rx*rz]);
    
    % scalar x vector
    verifyEqual(tc, ry * [rx ry rz], [ry*rx ry*ry ry*rz]);
    
    % vector x scalar
    verifyEqual(tc, [rx ry rz] * ry, [rx*ry ry*ry rz*ry]);
    
    %% SO3-vector product
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
    rx = SO3.Rx(pi/2);
    ry = SO3.Ry(pi/2);
    rz = SO3.Rz(pi/2);
    u = SO3();
    
    %% SO3-SO3 product
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
    
    rx = SO3.Rx(pi/2);
    ry = SO3.Ry(pi/2);
    rz = SO3.Rz(pi/2);
    u = SO3();
    
    % scalar / scalar
    % implicity tests inv

    verifyEqual(tc, rx/u, rx);
    verifyEqual(tc, ry/ry, u);

    % vector / vector
    verifyEqual(tc, [ry rz rx] / [rx ry rz], [ry/rx rz/ry rx/rz]);
    
    % vector / scalar
    verifyEqual(tc, [rx ry rz] / ry, [rx/ry ry/ry rz/ry]);
    
    % scalar /vector
    verifyEqual(tc, ry / [rx ry rz], [ry/rx ry/ry ry/rz]);
end

function divide_normalized_test(tc)
    
    rx = SO3.Rx(pi/2);
    ry = SO3.Ry(pi/2);
    rz = SO3.Rz(pi/2);
    u = SO3();
    
    % scalar / scalar
    verifyEqual(tc, double(rx./ry), double(rx/ry), 'AbsTol', 1e-10);

    % shouldn't make that much difference here
    verifyEqual(tc, double(rx./ry), double(rx/ry), 'AbsTol', 1e-10);
    verifyEqual(tc, double(rx./rz), double(rx/rz), 'AbsTol', 1e-10); 
    
    verifyEqual(tc, double(rx./u), double(rx), 'AbsTol', 1e-10);
    verifyEqual(tc, double(ry./ry), double(u), 'AbsTol', 1e-10);

    % vector / vector
    verifyEqual(tc, [ry rz rx] ./ [rx ry rz], [ry./rx rz./ry rx./rz], 'AbsTol', 1e-10);
    
    % vector / scalar
    verifyEqual(tc, [rx ry rz] ./ ry, [rx./ry ry./ry rz./ry], 'AbsTol', 1e-10);
    
   % scalar /vector
    verifyEqual(tc, ry ./ [rx ry rz], [ry./rx ry./ry ry./rz]);
end


function conversions_test(tc)
    
    %% 3 angle
    verifyEqual(tc, SO3.rpy( 0.1, 0.2, 0.3 ).torpy, [ 0.1, 0.2, 0.3], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.eul( 0.1, 0.2, 0.3 ).toeul, [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.rpy( 10, 20, 30, 'deg' ).R, rpy2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    verifyEqual(tc, SO3.eul( 10, 20, 30, 'deg' ).R, eul2r( 10, 20, 30, 'deg' ), 'AbsTol', 1e-10  );
    
    %% (theta, v)
    th = 0.2; v = unit([1 2 3]);
    a = SO3.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    
    [a,b] = SO3.angvec(th, v ).toangvec;
    verifyEqual(tc, a, th, 'AbsTol', 1e-10  );
    verifyEqual(tc, b, v, 'AbsTol', 1e-10  );
    
    %% quaternion
    
    R = rpy2r( 0.2, 0.3, 0.4);
    q = UnitQuaternion(R);
    RR = SO3(R);
    verifyClass(tc, RR.UnitQuaternion, 'UnitQuaternion');
    verifyEqual(tc, q, RR.UnitQuaternion);
    
    
    %%  SE3                     convert to SE3 class

    T = rpy2tr(0.2, 0.3, 0.4);
    RR = SO3(T);
    
    verifyClass(tc, RR.SE3, 'SE3');
    verifyEqual(tc, RR.SE3, SE3(T), 'AbsTol', 1e-10 );
    verifyEqual(tc, RR.T, T, 'AbsTol', 1e-10 );
    
    %% Lie stuff
    th = 0.3; v = [0 1 0];
    RR = SO3.angvec( th, v);
    verifyEqual(tc, RR.log, skew(v)*th);
end

function miscellany_test(tc)
    
    r = SO3.rpy( 0.1, 0.2, 0.3 );
    verifyEqual(tc, det(r), 1, 'AbsTol', 1e-10  );
    
    verifyEqual(tc, dim(r), 3);
    
    verifyEqual( tc, eig(r), eig(r.double) );
    
    verifyEqual( tc, isSE(r), false );

    
    verifyClass(tc, r.new, 'SO3');
    
    verifyClass(tc, SO3.check(r), 'SO3');
    verifyClass(tc, SO3.check( rotx(0.3) ), 'SO3');
    z = SO3.check(r);
    verifyEqual(tc, double(z), double(r));
    
    z = SO3.check(rotx(0.3));
    verifyEqual(tc, double(z), rotx(0.3));
    
    r = SO3;
    verifyEqual(tc, r.n, [1 0 0]');
    verifyEqual(tc, r.o, [0 1 0]');
    verifyEqual(tc, r.a, [0 0 1]');

    % compatibility functions
    verifyEqual(tc, tr2rpy( SO3.rpy( 0.1, 0.2, 0.3) ), [ 0.1, 0.2, 0.3], 'AbsTol', 1e-10  );
    verifyEqual(tc, tr2eul( SO3.eul( 0.1, 0.2, 0.3  ), [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
    verifyEqual(tc, tr2rpy( SO3.rpy( 10, 20, 30, 'deg'), 'deg'), [ 10, 20, 30], 'AbsTol', 1e-10  );
    verifyEqual(tc, tr2eul( SO3.eul( 10, 20, 30, 'deg'), 'deg'), [ 0.1, 0.2, 0.3 ], 'AbsTol', 1e-10  );
end

function display_test(tc)
    
    r = SO3.rpy( 0.1, 0.2, 0.3 );
    
    r.print
    trprint(r)   % old style syntax
    
    r.plot
    
    r.animate
end