
%% This is for testing the Quaternion functions in the robotics Toolbox
function tests = QuaternionTest
  tests = functiontests(localfunctions);
end

function constructor_test(tc)
    
    verifyClass(tc, Quaternion(), 'Quaternion');
    
    verifyEqual(tc, Quaternion().double, [0 0 0 0]);
    
    %% from S
    verifyEqual(tc, Quaternion([1 0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, Quaternion([0 1 0 0]).double, [0 1 0 0]);
    verifyEqual(tc, Quaternion([0 0 1 0]).double, [0 0 1 0]);
    verifyEqual(tc, Quaternion([0 0 0 1]).double, [0 0 0 1]);
 
    verifyEqual(tc, Quaternion([2 0 0 0]).double, [2 0 0 0]);
    verifyEqual(tc, Quaternion([-2 0 0 0]).double, [-2 0 0 0]);

    %% from [S,V]
    verifyEqual(tc, Quaternion(1, [0 0 0]).double, [1 0 0 0]);
    verifyEqual(tc, Quaternion(0, [1 0 0]).double, [0 1 0 0]);
    verifyEqual(tc, Quaternion(0, [0 1 0]).double, [0 0 1 0]);
    verifyEqual(tc, Quaternion(0, [0 0 1]).double, [0 0 0 1]);
 
    verifyEqual(tc, Quaternion(2, [0 0 0]).double, [2 0 0 0]);
    verifyEqual(tc, Quaternion(-2, [0 0 0]).double, [-2 0 0 0]);
    
    %% pure
    v = [5 6 7];
    verifyEqual(tc, Quaternion.pure(v).double, [0 v],'absTol',1e-4);   
    
    %% copy constructor
    q = Quaternion([1 2 3 4]);
    verifyEqual(tc, Quaternion(q), q, 'AbsTol', 1e-10);

end

function primitive_convert_test(tc)
    % char
    
    s = char( Quaternion() );
    
    %% s,v
    verifyEqual(tc, Quaternion([1 0 0 0]).s, 1);
    verifyEqual(tc, Quaternion([1 0 0 0]).v, [0 0 0]);
    
    verifyEqual(tc, Quaternion([0 1 0 0]).s, 0);
    verifyEqual(tc, Quaternion([0 1 0 0]).v, [1 0 0]);
    
    verifyEqual(tc, Quaternion([0 0 1 0]).s, 0);
    verifyEqual(tc, Quaternion([0 0 1 0]).v, [0 1 0]);
    
    verifyEqual(tc, Quaternion([0 0 0 1]).s, 0);
    verifyEqual(tc, Quaternion([0 0 0 1]).v, [0 0 1]);

end

function resulttype_test(tc)
    
    q = Quaternion([2 0 0 0]);
    
    verifyClass(tc, q*q, 'Quaternion');

    % other combos all fail, test this?
    
    verifyClass(tc, q/q, 'Quaternion');

    verifyClass(tc, conj(q), 'Quaternion');
    verifyClass(tc, inv(q), 'Quaternion');
    verifyClass(tc, unit(q), 'UnitQuaternion');
    
    verifyClass(tc, q+q, 'Quaternion');
    verifyClass(tc, q-q, 'Quaternion');
end

function multiply_test(tc)
    
    q1 = Quaternion([1 2 3 4]);
    q2 = Quaternion([4 3 2 1]);
    q3 = Quaternion([-1 2 -3 4]);

    u = Quaternion([1 0 0 0]);
    
    %% quat-quat product
    % scalar x scalar
    
    verifyEqual(tc, q1*u, q1);
    verifyEqual(tc, u*q1, q1); 
    
    % vector x vector
    verifyEqual(tc, [q1 u q2 u q3 u] * [u q1 u q2 u q3], [q1 q1 q2 q2 q3 q3]);
    
    % scalar x vector
    verifyEqual(tc, q1 * [q1 q2 q3], [q1*q1 q1*q2 q1*q3]);
    
    % vector x scalar
    verifyEqual(tc, [q1 q2 q3] * q2, [q1*q2 q2*q2 q3*q2]);
    
    %% quat-real product
    % scalar x scalar
    
    v1 = q1.double;
    verifyEqual(tc, double(q1*5), v1*5, 'AbsTol', 1e-10);
    verifyEqual(tc, double(6*q1), v1*6, 'AbsTol', 1e-10);
    verifyEqual(tc, double(-2*q1), -2*v1, 'AbsTol', 1e-10);
    
    % scalar x vector
    verifyEqual(tc, 5*[q1 q2 q3], [5*q1 5*q2 5*q3], 'AbsTol', 1e-10);
    
    % vector x scalar
    verifyEqual(tc, [q1 q2 q3] * 5, [5*q1 5*q2 5*q3], 'AbsTol', 1e-10);
    
    %% matrix form of multiplication
    verifyEqual(tc, q1.mat44 * q2.double', double(q1*q2)' );


end


function divide_test(tc)

    q1 = Quaternion([1 2 3 4]);
    q2 = Quaternion([4 3 2 1]);
    q3 = Quaternion([-1 2 -3 4]);
    
    u = Quaternion([1 0 0 0]);
    
        % scalar / scalar

    verifyEqual(tc, q1/u, q1, 'AbsTol', 1e-10);
    verifyEqual(tc, q2/u, q2, 'AbsTol', 1e-10);
    verifyEqual(tc, q1/q1, u, 'AbsTol', 1e-10);
    
    verifyEqual(tc, q2/q2, u, 'AbsTol', 1e-10);

    % vector / vector
    verifyEqual(tc, [q1 q2 q3] / [q2 q3 q1], [q1/q2 q2/q3 q3/q1]);
    
    % vector / scalar
    verifyEqual(tc, [q1 q2 q3] / q2, [q1/q2 q2/q2 q3/q2]);    
end

    
function equality_test(tc)
    q1 = Quaternion([1 2 3 4]);
    q2 = Quaternion([-2 1 -4 3]);

    verifyTrue(tc, q1 == q1);
    verifyFalse(tc, q1 == q2);

    verifyTrue(tc, q1 ~= q2);
    verifyFalse(tc, q2 ~= q2);

    R = rotx(0.1);
    RR = cat(3, R,R,R);
    qt1 = [q1 q1 q2 q2];
    qt2 = [q1 q2 q2 q1];

    verifyEqual(tc, qt1==q1, [1 1 0 0]);
    verifyEqual(tc, q1==qt1, [1 1 0 0]);
    verifyEqual(tc, qt1==qt1, [1 1 1 1]);

    verifyEqual(tc, qt2==q1, [1 0 0 1]);
    verifyEqual(tc, q1==qt2, [1 0 0 1]);
    verifyEqual(tc, qt1==qt2, [1 0 1 0]);

    verifyEqual(tc, qt1~=q1, [0 0 1 1]);
    verifyEqual(tc, q1~=qt1, [0 0 1 1]);
    verifyEqual(tc, qt1~=qt1, [0 0 0 0]);

    verifyEqual(tc, qt2~=q1, [0 1 1 0]);
    verifyEqual(tc, q1~=qt2, [0 1 1 0]);
    verifyEqual(tc, qt1~=qt2, [0 1 0 1]);
end

function basic_multiply_test(tc)
    % test run multiplication tests on quaternions
    q = Quaternion([1 0 0 0]) * Quaternion([1 0 0 0]);
    verifyEqual(tc, q.double, [1 0 0 0 ],'absTol',1e-4);
    
    q = Quaternion([1 0 0 0]) * Quaternion([1 2 3 4]);
    verifyEqual(tc, q.double, [1 2 3 4],'absTol',1e-4);
    
    q = Quaternion([1 2 3 4]) * Quaternion([1 2 3 4]);
    verifyEqual(tc, q.double, [-28 4 6 8],'absTol',1e-4);    
end
    
function add_sub_test(tc)
    v1 = [1 2 3 4]; v2 = [2 2 4 7];
    q = Quaternion(v1) + Quaternion(v2);
    
    verifyEqual(tc, q.double, v1+v2, 'absTol',1e-4);
    q = Quaternion(v1) - Quaternion(v2);
    
    verifyEqual(tc, q.double, v1-v2, 'absTol',1e-4);
end

    
function inverse_test(tc)
    
    u = Quaternion([1 0 0 0]); % unit quaternion
    q = Quaternion([1 2 3 4]);
    
    verifyEqual(tc, q*inv(q), u, 'AbsTol', 1e-10  );
    verifyEqual(tc, inv(q)*q, u, 'AbsTol', 1e-10  );
end

function power_test(tc)

    q = Quaternion([1 2 3 4]);
    
    verifyEqual(tc, q^0, Quaternion([1 0 0 0]), 'AbsTol', 1e-10  );
    verifyEqual(tc, q^(-1), inv(q), 'AbsTol', 1e-10  );
    verifyEqual(tc, q^2, q*q, 'AbsTol', 1e-10  );

end


function miscellany_test(tc)
    v = [1 2 3 4];
    q = Quaternion(v);
    u = Quaternion([1 0 0 0]);
    
    %% norm
    verifyEqual(tc, q.norm, norm(v), 'AbsTol', 1e-10  );
    verifyEqual(tc, norm([q u q]), [norm(v) 1 norm(v)]', 'AbsTol', 1e-10  );
    
    %% unit
    qu = q.unit();
    verifyEqual(tc, qu.double, v/norm(v), 'AbsTol', 1e-10  );
    verifyEqual(tc, unit([q u q]), [qu u qu], 'AbsTol', 1e-10  );
    
    %% inner
    verifyEqual(tc, u.inner(u), 1, 'AbsTol', 1e-10  );
    verifyEqual(tc, q.inner(q), q.norm^2, 'AbsTol', 1e-10  );
    verifyEqual(tc, q.inner(u), dot(q.double, u.double), 'AbsTol', 1e-10  );


end


%  dot                     derivative of quaternion with angular velocity w

%  angle                   angle between two quaternions
%  plot                    plot a coordinate frame representing orientation of quaternion
%  animate                 animates a coordinate frame representing changing orientation
%                          of quaternion sequence


% %    Quaternion                 - constructor
% function Quaternion_test(tc)
%     q = Quaternion([1 2 3 4]);
%     verifyEqual(tc, q.double(),[1 2 3 4],'absTol',1e-4);
%     verifyEqual(tc, q.s, 1,'absTol',1e-4);
%     verifyEqual(tc, q.v,[2 3 4],'absTol',1e-4);
% 
%     % Q = Quaternion(Q1) is a copy of the quaternion Q1
%     q2 = Quaternion(q);
%     verifyEqual(tc, q.double(),[1 2 3 4],'absTol',1e-4);
%     
% 
%     % Q = Quaternion([S V1 V2 V3]) specifying directly its 4 elements
%     q = Quaternion([1 2 3 4]);
%     verifyEqual(tc, q.double,[1 2 3 4],'absTol',1e-4);
%     
%     q = Quaternion('component', [1 2 3 4]);
%     verifyEqual(tc, q.double,[1 2 3 4],'absTol',1e-4);
% 
%     % Q = Quaternion(S)  scalar S and zero vector part: S<0,0,0>
%     q = Quaternion(2);
%     verifyEqual(tc, q.double, [2 0 0 0],'absTol',1e-4);
% 
%     % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
%     q = Quaternion([1 2 3]);
%     verifyEqual(tc, q.double(),[0 1 2 3],'absTol',1e-4);    
% 
% 
%     
%     % Q = Quaternion(V, TH) is a unit quaternion corresponding to rotation of TH about the vector V.
%     Th = pi;         
%     q = Quaternion(Th,[1 2 3]);
%     expected_out=[0.0000 0.2673 0.5345 0.8018];
%     verifyEqual(tc, q.double(), expected_out,'absTol',1e-4);
%     
%     %test for input errors!! 
%     verifyError(tc, @()Quaternion([1 2 3 4 5]),'RTB:Quaternion:badarg');
% end
