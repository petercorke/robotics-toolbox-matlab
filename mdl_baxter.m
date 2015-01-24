%MDL_BAXTER Kinematic model of Baxter dual-arm robot
%
% MDL_BAXTER is a script that creates the workspace variables LEFT and
% RIGHT which describes the kinematic characteristics of the two arms of a
% Rethink Robotics Baxter robot using standard DH conventions.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qd         lower arm horizontal as per data sheet
%
% Notes::
% - SI units of metres are used.
%
% References::
% "Kinematics Modeling and Experimental Verification of Baxter Robot"
% Z. Ju, C. Yang, H. Ma, Chinese Control Conf, 2015.
%
% See also SerialLink, mdl_nao.


% MODEL: Baxter, Rethink Robotics, 7DOF, standard_DH


%     th  d  a  alpha

L(1) = Revolute('d', 0.27,        'a', 0.069, 'alpha', -pi/2);
L(2) = Revolute('d', 0,           'a', 0, 'alpha', pi/2, 'offset', pi/2);
L(3) = Revolute('d', 0.102+0.262, 'a', 0.069, 'alpha', -pi/2);
L(4) = Revolute('d', 0,           'a', 0, 'alpha', pi/2);
L(5) = Revolute('d', 0.103+0.271, 'a', 0.010, 'alpha', -pi/2);
L(6) = Revolute('d', 0,           'a', 0, 'alpha', pi/2);
L(7) = Revolute('d', 0.28,        'a', 0, 'alpha', 0);

left =  SerialLink(L, 'name', 'Baxter LEFT');
right = SerialLink(L, 'name', 'Baxter RIGHT');

left.base = transl(0.064614, 0.25858, 0.119)*rpy2tr(0, 0, pi/4);
right.base = transl(0.063534, -0.25966, 0.119)*rpy2tr(0, 0, -pi/4);

% define the workspace vectors:
%   qz         zero joint angle configuration
%   qr         vertical 'READY' configuration
%   qstretch   arm is stretched out in the X direction
%   qn         arm is at a nominal non-singular configuration
%
qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0 0];
qn = [0 pi/4 pi/2 0 pi/4  0 0];
