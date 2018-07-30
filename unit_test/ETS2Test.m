function tests = ETS2Test
    tests = functiontests(localfunctions);
    clc
end

function revolute1_test(tc)
    import ETS2.*
    
    a1 = 1;
    E = Rz('q1') * Tx(a1);
    
    tc.verifyTrue(isa(E, 'ETS2'))
    tc.verifySize(E, [1 2])
    tc.verifyEqual(E.n, 1)  % number of joints
    tc.verifyClass(char(E), 'char')
    
    tc.verifyEqual(E.structure, 'R');
end

function revolute2_test(tc)
    import ETS2.*
    
    a1 = 1; a2 = 1;
    E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
    
    tc.verifyTrue(isa(E, 'ETS2'))
    tc.verifySize(E, [1 4])
    tc.verifyEqual(E.n, 2)  % number of joints
    tc.verifyClass(char(E), 'char')
    
    tc.verifyEqual(E.structure, 'RR');
end

function prismatic_test(tc)
    import ETS2.*
    
    a1 = 1;
    E = Rz('q1') * Tx(a1) * Tx('q2');
    tc.verifyTrue(isa(E, 'ETS2'))
    tc.verifySize(E, [1 3])
    tc.verifyEqual(E.n, 2)  % number of joints
    tc.verifyClass(char(E), 'char')
    
    tc.verifyEqual(E.structure, 'RP');
end

function fkine_test(tc)
    import ETS2.*
    
    a1 = 1;
    E = Rz('q1') * Tx(a1) * Tx('q2');
    
    T = E.fkine([pi/2 1]);
    tc.verifyClass(T, 'SE2')
    tc.verifyEqual(double(T), [0 -1 0; 1 0 2; 0 0 1], 'AbsTol', 1e-12)
end

function plot_test(tc)
    import ETS2.*
    
    a1 = 1; a2 = 1;
    E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
    
    E.plot([pi/4 -pi/4])
end