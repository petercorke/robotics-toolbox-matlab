function tests = ETS3Test
    tests = functiontests(localfunctions);
    clc
end

function revolute1_test(tc)
    import ETS3.*
    
    a1 = 1;
    E = Rx('q1') * Tx(a1) * Ry('q2') * Ty(a1) * Rz('q3');
    
    tc.verifyTrue(isa(E, 'ETS3'))
    tc.verifySize(E, [1 5])
    tc.verifyEqual(E.n, 3)  % number of joints
    tc.verifyClass(char(E), 'char')
    
    tc.verifyEqual(E.structure, 'RRR');
end

function prismatic_test(tc)
    import ETS3.*
    
    a1 = 1;
    E = Tx('q1') * Rx(a1) * Ty('q2') * Ry(a1) * Tz('q3');
    
    tc.verifyTrue(isa(E, 'ETS3'))
    tc.verifySize(E, [1 5])
    tc.verifyEqual(E.n, 3)  % number of joints
    tc.verifyClass(char(E), 'char')
    
    tc.verifyEqual(E.structure, 'PPP');
end

function fkine_test(tc)
    import ETS3.*
    
    a1 = 1;
    E = Rx('q1') * Tx(a1) * Ry('q2') * Ty(a1) * Rz('q3');
    
    T = E.fkine([pi/2 pi/2 pi/2]);
    tc.verifyClass(T, 'SE3')
    tc.verifyEqual(double(T), [0 0 1 1; 0 -1 0 0; 1 0 0 1; 0 0 0 1], 'AbsTol', 1e-12)
end

function plot_test(tc)
    import ETS3.*
    
    a1 = 1; a2 = 1;
    E = Rx('q1') * Tx(a1) * Ry('q2') * Ty(a1) * Rz('q3');
    
    E.plot([pi/4 -pi/4 pi/4])
end