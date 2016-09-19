% Jacobians for bundle adjustment

% intrinsics
syms u0 v0 rho f real
K = [f/rho 0 u0; 0 f/rho v0; 0 0 1]

syms vx vy vz theta

% world point
% syms Px Py Pz P
% P = [Px Py Pz].'
P = sym('P', [3,1], 'real');

% camera translation
t = sym('T', [3,1], 'real');

% camera rotation
syms qs qx qy qz real

qs = sqrt(1-qx^2 - qy^2 - qz^2);


% inverse R
R = UnitQuaternion.q2r([qs qx qy qz]);


% camera projection model
%  - homogeneous
uvw = K * (R'*P - R'*t);

%  - Euclidean
uv = uvw(1:2) ./ uvw(3);

uv = simplify(uv)

A = jacobian(uv, [t' qx qy qz]);

B = jacobian(uv, P);


% matlabFunction(A, 'file', 'jacA.m');
% matlabFunction(B, 'file', 'jacB.m');
% matlabFunction(uv, 'file', 'proj.m');

%% quaternion vector update
matlabFunction(uv, A, B, 'file', 'cameraModel', 'Vars', [t(1) t(2) t(3) qx qy qz P(1) P(2) P(3) f rho u0 v0])

syms qs2 qx2 qy2 qz2 real
qs2 = sqrt(1-qx2^2 - qy2^2 - qz2^2);

Q1 = Quaternion([qs qx qy qz]);
Q2 = Quaternion([qs2 qx2 qy2 qz2]);
QQ = Q1 * Q2;
QQ.v

matlabFunction(QQ.v, 'file', 'qvmul', 'Vars', [qx qy qz qx2 qy2 qz2]);




