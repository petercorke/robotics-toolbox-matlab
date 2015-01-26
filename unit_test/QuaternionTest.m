
%% This is for testing the Quaternion functions in the robotics Toolbox
function tests = QuaternionTest
  tests = functiontests(localfunctions);
end

% Quaternion
%    Quaternion                 - constructor
function Quaternion_test(testCase)
    % Q = Quaternion(Q1) is a copy of the quaternion Q1
    q = Quaternion().double();
    verifyEqual(testCase, q,[1 0 0 0],'absTol',1e-4);
    %
    % Q = Quaternion([S V1 V2 V3]) specifying directly its 4 elements
    q = Quaternion([1 2 3 4]).double();
    verifyEqual(testCase, q,[1 2 3 4],'absTol',1e-4);
    %
    % Q = Quaternion(S)  scalar S and zero vector part: S<0,0,0>
    q = Quaternion(2).double();
    verifyEqual(testCase, q,[2 0 0 0],'absTol',1e-4);
    %
    % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
    q = Quaternion([1 2 3]).double();
    verifyEqual(testCase, q,[0 1 2 3],'absTol',1e-4);    
    %
    % Q = Quaternion(V, TH) is a unit quaternion corresponding to rotation of TH about the vector V.
    Th = pi;         
    q = Quaternion(Th,[1 2 3]).double();
    expected_out=[0.0000 0.2673 0.5345 0.8018];
    verifyEqual(testCase, q,expected_out,'absTol',1e-4);
    
    % Q = Quaternion(R) is a unit quaternion corresponding to the orthonormal rotation matrix R.
    R = [1.0000         0         0         
         0    0.5403   -0.8415
         0    0.8415    0.5403];
    q = Quaternion(R).double();
    verifyEqual(testCase, q, [0.8776 0.4794 0 0],'absTol',1e-4);

    % constructor with vector argument
    Rs = cat(3, R, R, R);
    qr = Quaternion(Rs);
    verifySize(testCase, qr, [1 3]);
    
    % Q = Quaternion(T) is a unit quaternion equivalent to the rotational
    % part of the homogeneous transform T.
    TR = [0.5403         0    0.8415         0
              0    1.0000         0         0
        -0.8415         0    0.5403         0
              0         0         0    1.0000];         
    qt = Quaternion(TR).double();
    verifyEqual(testCase, qt, [0.8776 0 0.4794 0],'absTol',1e-4);

    % constructor with vector argument
    Ts = cat(3, TR, TR, TR);
    q = Quaternion(Ts);
    verifySize(testCase, q, [1 3]);

    %test for input errors!! 
    verifyError(testCase, @()Quaternion([1 2 3 4 5]),'RTB:Quaternion:badarg');
end
    
%    /                          - divide quaternion by quaternion or scalar
function Quaternion_Divide_test(testCase)
    % test run devision tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1/q2);
    expected_out = [1 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1/q2);
    expected_out = [0 -1 -2 -3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end
    
%    *                          - multiply quaternion by a quaternion or vector
function quaternion_compare(testCase)
    q1 = Quaternion(rotx(0.1));
    q2 = Quaternion(roty(0.1));

    verifyTrue(testCase, q1 == q1);
    verifyFalse(testCase, q1 == q2);

    verifyTrue(testCase, q1 ~= q2);
    verifyFalse(testCase, q1 ~= q2);

    R = rotx(0.1);
    RR = cat(3, R,R,R);
    qt1 = Quaternion(RR);
    R = roty(0.1);
    RR = cat(3, R,R,R);
    qt2 = Quaternion(RR);

    verifyEqual(testCase, qt1==q1, [1 1 1]);
    verifyEqual(testCase, q1==qt1, [1 1 1]);
    verifyEqual(testCase, qt1==qt1, [1 1 1]);

    verifyEqual(testCase, qt2==q1, [0 0 0]);
    verifyEqual(testCase, q1==qt2, [0 0 0]);
    verifyEqual(testCase, qt1==qt2, [0 0 0]);

    verifyEqual(testCase, qt1~=q1, [0 0 0]);
    verifyEqual(testCase, q1~=qt1, [0 0 0]);
    verifyEqual(testCase, qt1~=qt1, [0 0 0]);

    verifyEqual(testCase, qt2~=q1, [1 1 1]);
    verifyEqual(testCase, q1~=qt2, [1 1 1]);
    verifyEqual(testCase, qt1~=qt2, [1 1 1]);
end

function Quaternion_Multiply_test(testCase)
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1*q2);
    expected_out = [1 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1*q2);
    expected_out = [0 1 2 3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    %q*v       rotate vector by quaternion, v is 3x1
    q1 = Quaternion([1 2 3]);
    out = (q1*[3 2 1]);
    expected_out = [-22;12;46];
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end
    
%  q+q2      return elementwise sum of quaternions
function Quaternion_Addition_test(testCase)
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1+q2);
    expected_out = [2 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1+q2);
    expected_out = [1 1 2 3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

%  q-q2      return elementwise difference of quaternions
function Quaternion_Subtraction_test(testCase)
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1-q2);
    expected_out = [0 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1-q2);
    expected_out = [1 -1 -2 -3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

%  q^n       return q to power n (integer only)
function Quaternion_Power_test(testCase)
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    out = (q1^2);
    expected_out = [1 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion([1 2 3]);
    out = (q1^3);
    expected_out = [0 -14 -28 -42];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end
    
%    inv                        - invert a quaternion
function Quaternion_Invert_test(testCase)
    % test run inversion tests on quaternions
    q1 = Quaternion();
    out = q1.inv;
    expected_out = [1 0 0 0];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.inv;
    expected_out = [0 -1 -2 -3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

%    norm                       - norm of a quaternion
function Quaternion_Normal_test(testCase)
    % test run Normal tests on quaternions
    q1 = Quaternion();
    out = q1.norm;
    expected_out = 1;
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.norm;
    expected_out = 3.7417;
    verifyEqual(testCase, out,expected_out,'absTol',1e-4);
end

%    unit                       - unitize a quaternion
function Quaternion_Unit_test(testCase)
    % test run Normal tests on quaternions
    q1 = Quaternion();
    out = q1.unit;
    expected_out = [1 0 0 0];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.unit;
    expected_out = [0 0.2673 0.5345 0.8018];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

%  interp    interpolation (slerp) between q and q2, 0<=s<=1
function Quaternion_Interpolation_test(testCase)
    % test run Interpolation tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = q1.interp(q2,1);
    expected_out = [1 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = q1.interp(q2,1);
    expected_out = [0 1 2 3];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end


%  scale     interpolation (slerp) between identity and q, 0<=s<=1
function Quaternion_Scale_test(testCase)
    % test run Interpolation tests on quaternions
    q1 = Quaternion();
    out = q1.scale(0.5);
    expected_out = [1 0 0 0 ];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.scale(0.5);
    expected_out = [0.2582 0.2582 0.5164 0.7746];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

    %  dot       derivative of quaternion with angular velocity w
function Quaternion_Dot_test(testCase)
    % test run dot tests on quaternions
    q1 = Quaternion();
    omega = [1 2 3];
    out = q1.dot(omega);
    expected_out = [0    0.5000    1.0000    1.5000];
    verifyEqual(testCase, out.double,expected_out,'absTol',1e-4);
end

%  plot function of(testCase) Quaternion_test
function plot_test(testCase)
    R = [1.0000         0         0         
         0    0.5403   -0.8415         
         0    0.8415    0.5403];         
    q = Quaternion(R).double;
    plot(q);
end
