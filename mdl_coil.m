%MDL_COIL Create model of a coil manipulator
%
% MDL_COIL creates the workspace variable coil which describes the
% kinematic characteristics of a serial link manipulator with 50 joints
% that folds into a helix shape.
%
% MDL_BALL(N) as above but creates a manipulator with N joints.
%
% Also defines the workspace vectors:
%   q  joint angle vector for default helical configuration
%
% Reference::
% - "A divide and conquer articulated-body algorithm for parallel O(log(n))
%   calculation of rigid body dynamics, Part 2",
%   Int. J. Robotics Research, 18(9), pp 876-892. 
%
% Notes::
% - Unlike most other mdl_xxx scripts this one is actually a function that
%   behaves like a script and writes to the global workspace.
%
% See also SerialLink, mdl_ball.

% MODEL: generic, coil, hyper redundant, 50DOF, standard_DH

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
function r = mdl_coil(N)
    
    if nargin == 0
        N = 50;
    end
    
    % create the links
    for i=1:N
        links(i) = Link('d', 0, 'a', 1/N, 'alpha', 5*pi/N);
    end
    
    % and build a serial link manipulator
    robot = SerialLink(links, 'name', 'coil');
    
    % place the variables into the global workspace
    if nargin == 1
        r = robot;
    elseif nargin == 0
        assignin('base', 'coil', robot);
        assignin('base', 'q', 10*pi/N*ones(1,N));
    end
