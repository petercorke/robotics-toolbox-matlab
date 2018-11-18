function tests = SO2Test
  tests = functiontests(localfunctions);
end


% we will assume that the primitives rotx,trotx, etc. all work


function constructor_test(tc)
    
    tc.verifyEqual(SO2().double, eye(2,2));
    
    %% from angle
    
    tc.verifyEqual(SO2(0).double, eye(2,2), 'AbsTol', 1e-10  );
    tc.verifyEqual(SO2(pi/2).double, rot2(pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(SO2(90, 'deg').double, rot2(pi/2), 'AbsTol', 1e-10  );
   
    
    %% from R
    
    tc.verifyEqual(SO2( eye(2,2) ).double, eye(2,2), 'AbsTol', 1e-10  );

    tc.verifyEqual(SO2( rot2(pi/2) ).double, rot2(pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(SO2( rot2(pi) ).double, rot2(pi), 'AbsTol', 1e-10  );
   

    %% from T
    tc.verifyEqual(SO2( trot2(pi/2) ).double, rot2(pi/2), 'AbsTol', 1e-10  );
    tc.verifyEqual(SO2( trot2(pi) ).double, rot2(pi), 'AbsTol', 1e-10  );

    
    %% R,T
    tc.verifyEqual(SO2( eye(2,2) ).R, eye(2,2), 'AbsTol', 1e-10  );
   
    tc.verifyEqual(SO2( rot2(pi/2) ).R, rot2(pi/2), 'AbsTol', 1e-10  );
    
    
    %% vectorised forms of R
    R = [];
    for theta = [-pi/2 0 pi/2 pi]
        R = cat(3, R, rot2(theta));

    end
    tc.verifyEqual(SO2(R).R, R, 'AbsTol', 1e-10);
    
    %% copy constructor
    r = SO2(rot2(0.3));
    tc.verifyEqual(SO2(r), r, 'AbsTol', 1e-10);
    

end

function concat_test(tc)
    x = SO2();
    xx = [x x x x];
    
    tc.verifyClass(xx, 'SO2');
    tc.verifySize(xx, [1 4]);
end

function primitive_convert_test(tc)
    % char
    
    s = char( SO2() );
   
end

function staticconstructors_test(tc)
    
    %% exponential
    tc.verifyEqual(SO2.exp( skew(0.3) ).R, rot2(0.3), 'AbsTol', 1e-10  );

end

function isa_test(tc)
    
    verifyTrue(tc, SO2.isa(rot2(0)) );
    verifyTrue(tc, SO2.isa(rot2(0), 'valid') );

    verifyFalse(tc, SO2.isa(1) )
end

function resulttype_test(tc)
    
    r = SO2();
    verifyClass(tc, r, 'SO2');

    verifyClass(tc, r*r, 'SO2');
    

    verifyClass(tc, r/r, 'SO2');
    
    verifyClass(tc, inv(r), 'SO2');
    
end

function multiply_test(tc)
    
    vx = [1 0]'; vy = [0 1]';
    r0 = SO2(0);
    r1 = SO2(pi/2);
    r2 = SO2(pi);
    u = SO2();
    
    %% SO2-SO2 product
    % scalar x scalar
    
    tc.verifyEqual(r0*u, r0);
    tc.verifyEqual(u*r0, r0); 
    
    % vector x vector
    tc.verifyEqual([r0 r1 r2] * [r2 r0 r1], [r0*r2 r1*r0 r2*r1]);
    
    % scalar x vector
    tc.verifyEqual(r1 * [r0 r1 r2], [r1*r0 r1*r1 r1*r2]);
    
    % vector x scalar
    tc.verifyEqual([r0 r1 r2] * r2, [r0*r2 r1*r2 r2*r2]);
    
    %% SO2-vector product
    % scalar x scalar
    
    tc.verifyEqual(r1*vx, vy, 'AbsTol', 1e-10);
    
    % vector x vector
    tc.verifyEqual([r0 r1 r0] * [vy vx vx], [vy vy vx], 'AbsTol', 1e-10);
    
    % scalar x vector
    tc.verifyEqual(r1 * [vx vy -vx], [vy -vx -vy], 'AbsTol', 1e-10);
    
    % vector x scalar
    tc.verifyEqual([r0 r1 r2] * vy, [vy -vx -vy], 'AbsTol', 1e-10);
end


function divide_test(tc)
    
    r0 = SO2(0);
    r1 = SO2(pi/2);
    r2 = SO2(pi);
    u = SO2();
    
    % scalar / scalar
    % implicity tests inv

    tc.verifyEqual(r1/u, r1);
    tc.verifyEqual(r1/r1, u);

    % vector / vector
    tc.verifyEqual([r0 r1 r2] / [r2 r1 r0], [r0/r2 r1/r1 r2/r0]);
    
    % vector / scalar
    tc.verifyEqual([r0 r1 r2] / r1, [r0/r1 r1/r1 r2/r1]);
end

function conversions_test(tc)
    
    T = SO2(pi/2).SE2;
    verifyClass(tc, T, 'SE2');
    tc.verifyEqual(T.T, trot2(pi/2));

    
    %% Lie stuff
    th = 0.3; 
    RR = SO2(th);
    tc.verifyEqual(RR.log, skew(th), 'AbsTol', 1e-10 );

end

function miscellany_test(tc)
    
    r = SO2( 0.3 );
    tc.verifyEqual(det(r), 1, 'AbsTol', 1e-10  );
    
    tc.verifyEqual(dim(r), 2);
    
    verifyEqual( tc, eig(r), eig(r.double) );
    
    verifyEqual( tc, isSE(r), false );

    
    verifyClass(tc, r.new, 'SO2');
    
    verifyClass(tc, SO2.check(r), 'SO2');
    verifyClass(tc, SO2.check( rot2(0.3) ), 'SO2');
    z = SO2.check(r);
    tc.verifyEqual(double(z), double(r));
    
    z = SO2.check(rot2(0.3));
    tc.verifyEqual(double(z), rot2(0.3));
    
    T = r.SE2;
    verifyClass(tc, SO2.check(T), 'SO2');
end

function display_test(tc)
    
    R = SO2( 0.3 );
    
    R.print
    trprint(R)   % old style syntax
    
    R.plot
    trplot(R)   % old style syntax
    
    R2 = SO2(0.6);
    R.animate
    R.animate(R2)
    tranimate(R2)   % old style syntax
    tranimate(R, R2)   % old style syntax
    tranimate2(R2)   % old style syntax
    tranimate2(R, R2)   % old style syntax
end