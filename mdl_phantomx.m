%MDL_PHANTOMX Create model of PhantomX pincher manipulator
%
%      mdl_phantomx
%
% Script creates the workspace variable px which describes the 
% kinematic characteristics of a PhantomX Pincher Robot, a 4 joint hobby
% class  manipulator by Trossen Robotics.
%
% Also define the workspace vectors:
%   qz         zero joint angle configuration
%
% Notes::
% - uses standard DH conventions.
% - Tool centrepoint is middle of the fingertips
% - all translational units in mm
%
% Reference::
%
% - http://www.trossenrobotics.com/productdocs/assemblyguides/phantomx-basic-robot-arm.html

clear L
L(1) = Revolute('d', 40, 'alpha', -pi/2);
L(2) = Revolute('a', -105, 'alpha', pi, 'offset', pi/2);
L(3) = Revolute('a', -105);
L(4) = Revolute('a', -105);

% Note alpha_2 = pi, needed to account for rotation axes of joints 3 and 4 having
% opposite sign to joint 2.
%
% s='Rz(q1) Tz(L1) Ry(q2) Tz(L2) Ry(q3) Tz(L3) Ry(q4) Tz(L4)'
% DHFactor(s)

px = SerialLink(L, 'name', 'PhantomX', 'manufacturer', 'Trossen Robotics');
qz = [0 0 0 0];
px.tool = trotz(-pi/2) * trotx(pi/2);