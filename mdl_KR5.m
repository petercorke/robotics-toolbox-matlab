%MDL_KR5 Create model of Kuka KR5 manipulator
%
% MDL_KR5 is a script that creates the workspace variable mico which
% describes the kinematic characteristics of a Kuka KR5 manipulator using
% standard DH conventions.
%
% Also define the workspace vectors:
%   qk1        nominal working position 1
%   qk2        nominal working position 2
%   qk3        nominal working position 3
%
% Notes::
% - SI units of metres are used.
% - Includes 11.5cm tool in the z-direction
%
% Author::
% - Gautam sinha
%   Indian Institute of Technology, Kanpur.
%
% See also SerialLink, mdl_irb140, mdl_fanuc10l, mdl_motomanHP6, mdl_S4ABB2p8, mdl_puma560.

% MODEL: Kuka, KR5, 6DOF, standard_DH

%mdl_KR5 
%Define simplest line model for KUKA KR5 robot
%Contain DH parameters for KUKA KR5 robot
%All link lenghts and offsets are measured in cm
clear L
%            theta    d           a       alpha
L(1) = Link([0        0.4         0.18    pi/2]);
L(2) = Link([0        0.135       0.60    pi]);
L(3) = Link([0        0.135       0.12   -pi/2]);
L(4) = Link([0        0.62        0       pi/2]);
L(5) = Link([0        0           0      -pi/2]);
L(6) = Link([0        0           0       0]);

KR5=SerialLink(L, 'name', 'Kuka KR5');
KR5.tool=transl(0,0,0.115);
KR5.ikineType = 'kr5';
KR5.model3d = 'KUKA/KR5_arc';

qk1=[pi/4 pi/3 pi/4 pi/6 pi/4 pi/6];
qk2=[pi/4 pi/3 pi/6 pi/3 pi/4 pi/6];
qk3=[pi/6 pi/3 pi/6 pi/3 pi/6 pi/3];
