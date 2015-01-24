%MDL_NAO Create model of Aldebaran NAO humanoid robot
%
% MDL_NAO is a script that creates several workspace variables
%
%   leftarm         left-arm kinematics (4DOF)
%   rightarm        right-arm kinematics (4DOF)
%   leftleg         left-leg kinematics (6DOF)
%   rightleg        right-leg kinematics (6DOF)
%
% which are each SerialLink objects that describe the kinematic
% characteristics of the arms and legs of the NAO humanoid.
%
% Reference::
% - "Forward and Inverse Kinematics for the NAO Humanoid Robot",
%   Nikolaos Kofinas,
%   Thesis, Technical University of Crete
%   July 2012.
% - "Mechatronic design of NAO humanoid"
%   David Gouaillier etal.
%   IROS 2009, pp. 769-774.
%
% Notes::
% - SI units of metres are used.
% - The base transform of arms and legs are constant with respect to the 
%   torso frame, which is assumed to be the constant value when the robot 
%   is upright.  Clearly if the robot is walking these base transforms 
%   will be dynamic.
% - The first reference uses Modified DH notation, but doesn't explicitly
%   mention this, and the parameter tables have the wrong column headings
%   for Modified DH parameters.
% - TODO; add joint limits
% - TODO; add dynamic parameters
%
% See also SerialLink, Revolute.

% MODEL: Aldebaran, NAO, humanoid, 4DOF, standard_DH


% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

deg = pi/180;

% NAO constants (in mm)
NeckOffsetZ = 126.50;
ShoulderOffsetY = 98.00;
ElbowOffsetY = 15.00;
UpperArmLength = 105.00;
LowerArmLength = 55.95;
ShoulderOffsetZ = 100.00;
HandOffsetX = 57.75;
HipOffsetZ = 85.00;
HipOffsetY = 50.00;
ThighLength = 100.00;
TibiaLength = 102.90;
FootHeight = 45.19;
HandOffsetZ = 12.31;

% set some default plot options, base and shadow are not useful for a multi-arm plot
plotopts = {'nobase', 'noshadow'};


leftarm = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2, 'a', 0, 'modified')
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', -pi/2, 'modified')
    Revolute('d', UpperArmLength, 'alpha', -pi/2, 'a', 0, 'modified')
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'modified')
    ], ...
    'base', transl(0, ShoulderOffsetY+ElbowOffsetY, ShoulderOffsetZ), ...
    'tool', trotz(pi/2)*transl(HandOffsetX+LowerArmLength, 0, 0), ...
    'plotopt', plotopts, ...
    'name', 'left arm', 'manufacturer', 'Aldabaran');

rightarm = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/2, 'a', 0, 'modified')
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'offset', pi/2, 'modified')
    Revolute('d', -UpperArmLength, 'alpha', -pi/2, 'a', 0, 'modified')
    Revolute('d', 0, 'alpha', pi/2,  'a', 0, 'modified')
    ], ...
    'base', transl(0, -ShoulderOffsetY-ElbowOffsetY, ShoulderOffsetZ), ...
    'tool', trotz(pi/2)*transl(-HandOffsetX-LowerArmLength, 0, 0)*trotz(-pi), ...
    'plotopt', plotopts, ...
    'name', 'right arm', 'manufacturer', 'Aldabaran');


leftleg = SerialLink( [
    Revolute('d', 0, 'alpha', -3*pi/4, 'a', 0, 'offset', -pi/2, 'modified')
    Revolute('d', 0, 'alpha', -pi/2,   'a', 0, 'offset', pi/4, 'modified')
    Revolute('d', 0, 'alpha', pi/2,    'a', 0, 'modified')
    Revolute('d', 0, 'alpha', 0,       'a', -ThighLength, 'modified')
    Revolute('d', 0, 'alpha', 0,       'a', -TibiaLength, 'modified')
    Revolute('d', 0, 'alpha', -pi/2,   'a', 0, 'modified')
    ], ...
    'base', transl(0, HipOffsetY, -HipOffsetZ), ...
    'tool', trotz(pi)*troty(-pi/2)*transl(0, 0, -FootHeight), ...
    'plotopt', plotopts, ...
    'name', 'left leg', 'manufacturer', 'Aldabaran');

rightleg = SerialLink( [
    Revolute('d', 0, 'alpha', -pi/4, 'a', 0, 'offset', -pi/2, 'modified')
    Revolute('d', 0, 'alpha', -pi/2,   'a', 0, 'offset', -pi/4, 'modified')
    Revolute('d', 0, 'alpha', pi/2,    'a', 0, 'modified')
    Revolute('d', 0, 'alpha', 0,       'a', -ThighLength, 'modified')
    Revolute('d', 0, 'alpha', 0,       'a', -TibiaLength, 'modified')
    Revolute('d', 0, 'alpha', -pi/2,   'a', 0, 'modified')
    ], ...
    'base', transl(0, -HipOffsetY, -HipOffsetZ), ...
    'tool', trotz(pi)*troty(-pi/2)*transl(0, 0, -FootHeight), ...
    'plotopt', plotopts, ...
    'name', 'right leg', 'manufacturer', 'Aldabaran');


% plot the legs and arms in a nominal sized workspace
clf
leftleg.plot([0 0 0 0 0 0], 'workspace', 400*[-1 1 -1 1 -1 1]);
hold on
rightleg.plot([0 0 0 0 0 0], 'workspace', 400*[-1 1 -1 1 -1 1]);
leftarm.plot([0 0 0 0], 'workspace', 400*[-1 1 -1 1 -1 1]);
rightarm.plot([0 0 0 0], 'workspace', 400*[-1 1 -1 1 -1 1]);


    
