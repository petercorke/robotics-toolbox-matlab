
%% This is for testing the Quaternion functions in the robotics Toolbox
function test_suite = TestRobotToolboxQuaternion
  initTestSuite;
% Quaternion
%    Quaternion                 - constructor
function Test_Quaternion
    % Q = Quaternion(Q1) is a copy of the quaternion Q1
    q = Quaternion().double;
    assertElementsAlmostEqual(q,[1 0 0 0],'absolute',1e-4);
    %
    % Q = Quaternion([S V1 V2 V3]) specifying directly its 4 elements
    q = Quaternion([1 2 3 4]).double;
    assertElementsAlmostEqual(q,[1 2 3 4],'absolute',1e-4);
    %
    % Q = Quaternion(S)  scalar S and zero vector part: S<0,0,0>
    q = Quaternion(2).double;
    assertElementsAlmostEqual(q,[2 0 0 0],'absolute',1e-4);
    %
    % Q = Quaternion(V) is a pure quaternion with the specified vector part: 0<V>
    q = Quaternion([1 2 3]).double;
    assertElementsAlmostEqual(q,[0 1 2 3],'absolute',1e-4);    
    %
    % Q = Quaternion(V, TH) is a unit quaternion corresponding to rotation of TH about the vector V.
    Th = pi;         
    q = Quaternion(Th,[1 2 3]).double;
    expected_out=[0.0000 0.2673 0.5345 0.8018];
    assertElementsAlmostEqual(q,expected_out,'absolute',1e-4);
    
    % Q = Quaternion(R) is a unit quaternion corresponding to the orthonormal rotation matrix R.
    R = [1.0000         0         0         
         0    0.5403   -0.8415         
         0    0.8415    0.5403];         
    q = Quaternion(R).double;
    assertElementsAlmostEqual(q,[0.8776 0.4794 0 0],'absolute',1e-4);
    
    % Q = Quaternion(T) is a unit quaternion equivalent to the rotational
    % part of the homogeneous transform T.
    TR = [0.5403         0    0.8415         0
              0    1.0000         0         0
        -0.8415         0    0.5403         0
              0         0         0    1.0000];         
    q = Quaternion(TR).double;
    assertElementsAlmostEqual(q,[0.8776 0 0.4794 0],'absolute',1e-4);
    %test for input errors!! 
    assertExceptionThrown(@()Quaternion([1 2 3 4 5]),'');
    
%    /                          - divide quaternion by quaternion or scalar
function TestQuaternion_Devide
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
function TestQuaternion_Multiply
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
function TestQuaternion_Addition
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
function TestQuaternion_Subtraction
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
function TestQuaternion_Power
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
function TestQuaternion_Invert
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
function TestQuaternion_Normal
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
function TestQuaternion_Unit
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
function TestQuaternion_Interpolation
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
function TestQuaternion_Scale
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
function TestQuaternion_Dot
    % test run dot tests on quaternions
    q1 = Quaternion();
    omega = [1 2 3];
    out = q1.dot(omega);
    expected_out = [0    0.5000    1.0000    1.5000];
    assertElementsAlmostEqual(out.double,expected_out,'absolute',1e-4);

