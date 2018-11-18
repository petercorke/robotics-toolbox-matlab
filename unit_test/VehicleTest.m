function tests = VehicleTest
  tests = functiontests(localfunctions);
  clc
end


function Bicycle_constructor_test(tc)
    
    % default constructor
    v = Bicycle();
    
    % display
    v
    
    % char
    s = v.char()
    tc.verifyTrue( ischar(s) );
    
    % all options
    v = Bicycle( ...
        'steermax', 2, ...
        'accelmax', 3, ...
        'covar', [1 2;3 4], ...
        'speedmax', 5, ...
        'L',2.5, ... 
        'x0', [1 2 3], ... 
        'dt', 0.5, ...
        'rdim', 0.3 ...
        );
    
    tc.verifyEqual(v.steermax, 2);
    tc.verifyEqual(v.accelmax, 3);
    tc.verifyEqual(v.V, [1 2;3 4]);
    tc.verifyEqual(v.speedmax, 5);
    tc.verifyEqual(v.L, 2.5);
    tc.verifyEqual(v.x0, [1 2 3]');
    tc.verifyEqual(v.dt, 0.5);
    tc.verifyEqual(v.rdim, 0.3);

    v.init()
    tc.verifyEqual(v.x, [1 2 3]');
    
end

function Bicycle_deriv_test(tc)
    
    v = Bicycle('steermax', Inf);
    
    xd =v.deriv([], [0 0 0], [1 0]);
    tc.verifyEqual( xd, [1 0 0]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 pi/2], [1 0]);
    tc.verifyEqual( xd, [0 1 0]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 0], [0 1]);
    tc.verifyEqual( xd, [0 0 0]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 0], [1 pi/4]);
    tc.verifyEqual( xd, [1 0 1]', 'AbsTol', 1e-6);
    
    v = Bicycle('steermax', pi/4);
    xd =v.deriv([], [0 0 0], [1 100]);
    tc.verifyEqual( xd, [1 0 1]', 'AbsTol', 1e-6);
    
    v = Bicycle('speedmax', 1, 'steermax', Inf);
    xd =v.deriv([], [0 0 0], [100 pi/4]);
    tc.verifyEqual( xd, [1 0 1]', 'AbsTol', 1e-6);
    
    v = Bicycle('accelmax', 1);
    xd =v.deriv([], [0 0 0], [100 0]);
    tc.verifyEqual( xd, [v.dt 0 0]', 'AbsTol', 1e-6);
    for i=1:9
            xd =v.deriv([], [0 0 0], [100 0]);
    end
    tc.verifyEqual( xd, [1 0 0]', 'AbsTol', 1e-6);

end

function Bicycle_update_test(tc)
    
    v = Bicycle('dt', 1, 'speedmax', Inf, 'steermax', Inf);
    
    tc.verifyEqual( v.update([1 0]), [1 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [1 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.update([1 0]), [1 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [2 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.update([-2 0]), [2 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [0 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( size(v.x_hist,1), 3);
    
    v.init();
    tc.verifyEqual( size(v.x_hist,1), 0);
    
    tc.verifyEqual( v.update([0 1]), [0 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.update([1 pi/4]), [1 1], 'AbsTol', 1e-6);
end

function Bicycle_f_test(tc)
    
    v = Bicycle('steermax', Inf);
    
    tc.verifyEqual( v.f([0 0 0], [1 0], [0 0]), [1 0 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([2 3 0], [1 0], [0 0]), [3 3 0], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.f([0 0 0], [0 0], [1 0]), [1 0 0], 'AbsTol', 1e-6);

    tc.verifyEqual( v.f([0 0 0], [0 1], [0 0]), [0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([0 0 0], [sqrt(2) pi/4], [0 0]), [1 1 pi/4], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([0 0 0], [sqrt(2) 0], [0 pi/4]), [1 1 pi/4], 'AbsTol', 1e-6);
end

function Bicycle_jacobian_test(tc)
    
    v = Bicycle('steermax', Inf);
    
    tc.verifyEqual( v.Fx([0 0 0], [0 0]), [1 0 0; 0 1 0; 0 0 1], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.Fx([0 0 0], [2 0]), [1 0 0; 0 1 2; 0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fx([2 3 0], [2 0]), [1 0 0; 0 1 2; 0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fx([0 0 0], [3 pi/2]), [1 0 -3; 0 1 0; 0 0 1], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.Fv([0 0 0], [0 0]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([2 3 0], [0 0]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([0 0 0], [2 3]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([0 0 pi/2], [0 0]), [0 0; 1 0; 0 1], 'AbsTol', 1e-6);
end


function Unicycle_constructor_test(tc)
    
    % default constructor
    v = Unicycle();
    
    % display
    v
    
    % char
    s = v.char()
    tc.verifyTrue( ischar(s) );
    
    % all options
    v = Bicycle( ...
        'accelmax', 3, ...
        'covar', [1 2;3 4], ...
        'speedmax', 5, ...
        'L',2.5, ... 
        'x0', [1 2 3], ... 
        'dt', 0.5, ...
        'rdim', 0.3 ...
        );
    
    tc.verifyEqual(v.accelmax, 3);
    tc.verifyEqual(v.V, [1 2;3 4]);
    tc.verifyEqual(v.speedmax, 5);
    tc.verifyEqual(v.L, 2.5);
    tc.verifyEqual(v.x0, [1 2 3]');
    tc.verifyEqual(v.dt, 0.5);
    tc.verifyEqual(v.rdim, 0.3);

    v.init()
    tc.verifyEqual(v.x, [1 2 3]');
end

function Unicycle_deriv_test(tc)
    
    v = Unicycle();
    
    xd =v.deriv([], [0 0 0], [1 0]);
    tc.verifyEqual( xd, [1 0 0]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 pi/2], [1 0]);
    tc.verifyEqual( xd, [0 1 0]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 0], [0 1]);
    tc.verifyEqual( xd, [0 0 1]', 'AbsTol', 1e-6);
    
    xd =v.deriv([], [0 0 0], [1 1]);
    tc.verifyEqual( xd, [1 0 1]', 'AbsTol', 1e-6);
    
    
    v = Unicycle('speedmax', 1);
    xd =v.deriv([], [0 0 0], [100 0]);
    tc.verifyEqual( xd, [1 0 0]', 'AbsTol', 1e-6);
    
    v = Unicycle('accelmax', 1);
    xd =v.deriv([], [0 0 0], [100 0]);
    tc.verifyEqual( xd, [v.dt 0 0]', 'AbsTol', 1e-6);
    for i=1:9
            xd =v.deriv([], [0 0 0], [100 0]);
    end
    tc.verifyEqual( xd, [1 0 0]', 'AbsTol', 1e-6);
    
end

function Unicycle_update_test(tc)
    
    v = Unicycle('dt', 1, 'speedmax', Inf);
    
    tc.verifyEqual( v.update([1 0]), [1 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [1 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.update([1 0]), [1 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [2 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.update([-2 0]), [2 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.x, [0 0 0]', 'AbsTol', 1e-6);
    
    tc.verifyEqual( size(v.x_hist,1), 3);
    
    v.init();
    tc.verifyEqual( size(v.x_hist,1), 0);
    
    tc.verifyEqual( v.update([0 1]), [0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.update([2 1]), [2 1], 'AbsTol', 1e-6);
end

function Unicycle_f_test(tc)
    
    v = Unicycle();
    
    tc.verifyEqual( v.f([0 0 0], [1 0], [0 0]), [1 0 0], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([2 3 0], [1 0], [0 0]), [3 3 0], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.f([0 0 0], [0 0], [1 0]), [1 0 0], 'AbsTol', 1e-6);

    tc.verifyEqual( v.f([0 0 0], [0 1], [0 0]), [0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([0 0 0], [sqrt(2) pi/4], [0 0]), [1 1 pi/4], 'AbsTol', 1e-6);
    tc.verifyEqual( v.f([0 0 0], [sqrt(2) 0], [0 pi/4]), [1 1 pi/4], 'AbsTol', 1e-6);
end

function Unicycle_jacobian_test(tc)
    
    v = Unicycle();
    
    tc.verifyEqual( v.Fx([0 0 0], [0 0]), [1 0 0; 0 1 0; 0 0 1], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.Fx([0 0 0], [2 0]), [1 0 0; 0 1 2; 0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fx([2 3 0], [2 0]), [1 0 0; 0 1 2; 0 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fx([0 0 0], [3 pi/2]), [1 0 -3; 0 1 0; 0 0 1], 'AbsTol', 1e-6);
    
    tc.verifyEqual( v.Fv([0 0 0], [0 0]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([2 3 0], [0 0]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([0 0 0], [2 3]), [1 0; 0 0; 0 1], 'AbsTol', 1e-6);
    tc.verifyEqual( v.Fv([0 0 pi/2], [0 0]), [0 0; 1 0; 0 1], 'AbsTol', 1e-6);
end