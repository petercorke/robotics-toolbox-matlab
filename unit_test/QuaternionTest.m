
%% This is for testing the Quaternion functions in the robotics Toolbox
function test_suite = RobotToolboxQuaternion_test
  initTestSuite;

% Quaternion
%    Quaternion                 - constructor
function Quaternion_test
    % Q = Quaternion(Q1) is a copy of the quaternion Q1
    q = Quaternion().double();
    assertElementsAlmostEqual(q,[1 0 0 0],'absolute',1e-4);
    %
    % Q = Quaternion([S V1 V2 V3]) specifying directly its 4 elements
    q = Quaternion([1 2 3 4]).double();
    assertElementsAlmostEqual(q,[1 2 3 4],'absolute',1e-4);
    %
    % Q = Quaternion(S)  scalar S and zero vector part: S<0,0,0>
    q = Quaternion(2).double();
    assertElementsAlmostEqual(q,[2 0 0 0],'absolute',1e-4);
    %
    % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
    q = Quaternion([1 2 3]).double();
    assertElementsAlmostEqual(q,[0 1 2 3],'absolute',1e-4);    
    %
    % Q = Quaternion(V, TH) is a unit quaternion corresponding to rotation of TH about the vector V.
    Th = pi;         
    q = Quaternion(Th,[1 2 3]).double();
    expected_out=[0.0000 0.2673 0.5345 0.8018];
    assertElementsAlmostEqual(q,expected_out,'absolute',1e-4);
    
    % Q = Quaternion(R) is a unit quaternion corresponding to the orthonormal rotation matrix R.
    R = [1.0000         0         0         
         0    0.5403   -0.8415
         0    0.8415    0.5403];
    q = Quaternion(R).double();
    assertElementsAlmostEqual(q, [0.8776 0.4794 0 0],'absolute',1e-4);

    % constructor with vector argument
    Rs = cat(3, R, R, R);
    qr = Quaternion(Rs);
    assertIsSize(qr, [1 3]);
    
    % Q = Quaternion(T) is a unit quaternion equivalent to the rotational
    % part of the homogeneous transform T.
    TR = [0.5403         0    0.8415         0
              0    1.0000         0         0
        -0.8415         0    0.5403         0
              0         0         0    1.0000];         
    qt = Quaternion(TR).double();
    assertElementsAlmostEqual(qt, [0.8776 0 0.4794 0],'absolute',1e-4);

    % constructor with vector argument
    Ts = cat(3, TR, TR, TR);
    q = Quaternion(Ts);
    assertIsSize(q, [1 3]);

    %test for input errors!! 
    assertExceptionThrown(@()Quaternion([1 2 3 4 5]),'');
    
%    /                          - divide quaternion by quaternion or scalar
function Quaternion_Divide_test
    % test run devision tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1/q2);
    expected_out = [1 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1/q2);
    expected_out = [0 -1 -2 -3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    
%    *                          - multiply quaternion by a quaternion or vector
function quaternion_compare
    q1 = Quaternion(rotx(0.1));
    q2 = Quaternion(roty(0.1));

    assertTrue(q1 == q1);
    assertFalse(q1 == q2);

    assertTrue(q1 ~= q2);
    assertFalse(q1 ~= q2);

    R = rotx(0.1);
    RR = cat(3, R,R,R);
    qt1 = Quaternion(RR);
    R = roty(0.1);
    RR = cat(3, R,R,R);
    qt2 = Quaternion(RR);

    assertEqual(qt1==q1, [1 1 1]);
    assertEqual(q1==qt1, [1 1 1]);
    assertEqual(qt1==qt1, [1 1 1]);

    assertEqual(qt2==q1, [0 0 0]);
    assertEqual(q1==qt2, [0 0 0]);
    assertEqual(qt1==qt2, [0 0 0]);

    assertEqual(qt1~=q1, [0 0 0]);
    assertEqual(q1~=qt1, [0 0 0]);
    assertEqual(qt1~=qt1, [0 0 0]);

    assertEqual(qt2~=q1, [1 1 1]);
    assertEqual(q1~=qt2, [1 1 1]);
    assertEqual(qt1~=qt2, [1 1 1]);

function Quaternion_Multiply_test
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1*q2);
    expected_out = [1 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1*q2);
    expected_out = [0 1 2 3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    %q*v       rotate vector by quaternion, v is 3x1
    q1 = Quaternion([1 2 3]);
    out = (q1*[3 2 1]);
    expected_out = [-22;12;46];
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    
%  q+q2      return elementwise sum of quaternions
function Quaternion_Addition_test
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1+q2);
    expected_out = [2 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1+q2);
    expected_out = [1 1 2 3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

%  q-q2      return elementwise difference of quaternions
function Quaternion_Subtraction_test
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = (q1-q2);
    expected_out = [0 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = (q1-q2);
    expected_out = [1 -1 -2 -3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

%  q^n       return q to power n (integer only)
function Quaternion_Power_test
    % test run multiplication tests on quaternions
    q1 = Quaternion();
    out = (q1^2);
    expected_out = [1 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion([1 2 3]);
    out = (q1^3);
    expected_out = [0 -14 -28 -42];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    
%    inv                        - invert a quaternion
function Quaternion_Invert_test
    % test run inversion tests on quaternions
    q1 = Quaternion();
    out = q1.inv;
    expected_out = [1 0 0 0];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.inv;
    expected_out = [0 -1 -2 -3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
%    norm                       - norm of a quaternion
function Quaternion_Normal_test
    % test run Normal tests on quaternions
    q1 = Quaternion();
    out = q1.norm;
    expected_out = 1;
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.norm;
    expected_out = 3.7417;
    assertElementsAlmostEqual(out,expected_out,'absolute',1e-4);

%    unit                       - unitize a quaternion
function Quaternion_Unit_test
    % test run Normal tests on quaternions
    q1 = Quaternion();
    out = q1.unit;
    expected_out = [1 0 0 0];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.unit;
    expected_out = [0 0.2673 0.5345 0.8018];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

%  interp    interpolation (slerp) between q and q2, 0<=s<=1
function Quaternion_Interpolation_test
    % test run Interpolation tests on quaternions
    q1 = Quaternion();
    q2 = Quaternion();
    out = q1.interp(q2,1);
    expected_out = [1 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion();
    q2 = Quaternion([1 2 3]);
    out = q1.interp(q2,1);
    expected_out = [0 1 2 3];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);


%  scale     interpolation (slerp) between identity and q, 0<=s<=1
function Quaternion_Scale_test
    % test run Interpolation tests on quaternions
    q1 = Quaternion();
    out = q1.scale(0.5);
    expected_out = [1 0 0 0 ];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);
    q1 = Quaternion([1 2 3]);
    out = q1.scale(0.5);
    expected_out = [0.2582 0.2582 0.5164 0.7746];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

    %  dot       derivative of quaternion with angular velocity w
function Quaternion_Dot_test
    % test run dot tests on quaternions
    q1 = Quaternion();
    omega = [1 2 3];
    out = q1.dot(omega);
    expected_out = [0    0.5000    1.0000    1.5000];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

%  plot function of Quaternion_test
function plot_test
        R = [1.0000         0         0         
             0    0.5403   -0.8415         
             0    0.8415    0.5403];         
        q = Quaternion(R).double;
        plot(q);
