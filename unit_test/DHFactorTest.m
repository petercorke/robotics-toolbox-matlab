function tests = DHFactorTest
  tests = functiontests(localfunctions);
  clc
end
  
function constructor1_test(tc)
    dh = DHFactor('Tz(L1)');
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'Tz(L1)');
    tc.verifyEqual(  char(dh.command('R')), 'SerialLink([], ''name'', ''R'', ''base'', eye(4,4), ''tool'', eye(4,4), ''offset'', [])');

    dh = DHFactor('Tz(q1)');
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(0, q1, 0, 0)');
    tc.verifyEqual(  char(dh.command('R')), 'SerialLink([0, 0, 0, 0, 1; ], ''name'', ''R'', ''base'', eye(4,4), ''tool'', eye(4,4), ''offset'', [0 ])');
    
    dh = DHFactor('Rz(q1)');
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, 0, 0, 0)');
    tc.verifyEqual(  char(dh.command('R')), 'SerialLink([0, 0, 0, 0, 0; ], ''name'', ''R'', ''base'', eye(4,4), ''tool'', eye(4,4), ''offset'', [0 ])');
end
    
function constructor2_test(tc)
    s = 'Tz(L1) Rz(q1) Rx(90) Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, L1, L2, 90)');
    
    s = 'Tz(L1).Rz(q1).Rx(90).Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, L1, L2, 90)');
    
    s = 'Tz(L1)*Rz(q1)*Rx(90)*Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, L1, L2, 90)');
    
    s = 'Tz(-L1) Rz(q1) Rx(90) Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, -L1, L2, 90)');
    
    s = 'Tz(L1) Rz(-q1) Rx(90) Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(-q1, L1, L2, 90)');
    
    s = 'Tz(L1) Rz(q1) Rx(-90) Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, L1, L2, -90)');
    
    s = 'Tz(L1) Rz(q1) Rx(270) Tx(L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(q1, L1, L2, 270)');
    
    s = 'Tz(-L1) Rz(-q1) Rx(90) Tx(-L2)';
    dh = DHFactor(s);
    tc.verifyClass(dh, 'DHFactor');
    tc.verifySize(dh, [1 1]);
    tc.verifyEqual( char(dh.tool), 'eye(4,4)');
    tc.verifyEqual( char(dh.base), 'eye(4,4)');
    tc.verifyEqual( char(dh), 'DH(-q1, -L1, -L2, 90)');
end

function puma560_test(tc)
    s = 'Tz(L1) Rz(q1) Ry(q2) Ty(L2) Tz(L3) Ry(q3) Tx(L4) Ty(L5) Tz(L6) Rz(q4) Ry(q5) Rz(q6)'; 
    dh = DHFactor(s);
    tc.verifyEqual( char(dh), 'DH(q1, L1, 0, -90).DH(q2+90, 0, -L3, 0).DH(q3-90, L2+L5, L4, 90).DH(q4, L6, 0, -90).DH(q5, 0, 0, 90).DH(q6, 0, 0, 0)');
    L1 = 0; L2 = -0.2337; L3 = 0.4318; L4 = 0.0203; L5 = 0.0837; L6 = 0.4318;
    R = eval( dh.command('R') );
    tc.verifyEqual(R.n, 6);
end
