%MDL_KR5 Create model of Kuka KR5 manipulator
%
% MDL_KR5 is a script that creates the workspace variable KR5 which
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
% - Includes an 11.5cm tool in the z-direction
%
% Author::
% - Gautam Sinha,
%   Indian Institute of Technology, Kanpur.
%
% See also mdl_irb140, mdl_fanuc10l, mdl_motomanHP6, mdl_S4ABB2p8, mdl_puma560, SerialLink.

% MODEL: Kuka, KR5, 6DOF, standard_DH

%mdl_KR5 
%Define simplest line model for KUKA KR5 robot
%Contain DH parameters for KUKA KR5 robot
%All link lenghts and offsets are measured in cm
clear L
%            theta    d           a       alpha
links = [
	    Link([0        0.4         0.18    pi/2])
		Link([0        0.135       0.60    pi])
		Link([0        0.135       0.12   -pi/2])
		Link([0        0.62        0       pi/2])
		Link([0        0           0      -pi/2])
		Link([0        0           0       0])
	];

KR5=SerialLink(links, 'name', 'Kuka KR5');
KR5.tool=transl(0,0,0.115);
KR5.ikineType = 'kr5';
KR5.model3d = 'KUKA/KR5_arc';

qk1=[pi/4 pi/3 pi/4 pi/6 pi/4 pi/6];
qk2=[pi/4 pi/3 pi/6 pi/3 pi/4 pi/6];
qk3=[pi/6 pi/3 pi/6 pi/3 pi/6 pi/3];
qz = [0 0 0 0 0 0];
