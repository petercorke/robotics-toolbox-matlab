%Prismatic Robot manipulator prismatic link class
%
% A subclass of the Link class for a prismatic joint defined using standard
% Denavit-Hartenberg parameters: holds all information related to a robot
% link such as kinematics parameters, rigid-body inertial parameters, motor
% and transmission parameters.
%
% Constructors::
%  Prismatic      construct a prismatic joint+link using standard DH
%
% Information/display methods::
%  display       print the link parameters in human readable form
%  dyn           display link dynamic parameters
%  type          joint type: 'R' or 'P'
%
% Conversion methods::
%  char          convert to string
%
% Operation methods::
%  A             link transform matrix
%  friction      friction force
%  nofriction    Link object with friction parameters set to zero%
%
% Testing methods::
%  islimit       test if joint exceeds soft limit
%  isrevolute    test if joint is revolute
%  isprismatic   test if joint is prismatic
%  issym         test if joint+link has symbolic parameters
%
% Overloaded operators::
%  +             concatenate links, result is a SerialLink object
%
% Properties (read/write)::
%
%  theta        kinematic: joint angle
%  d            kinematic: link offset
%  a            kinematic: link length
%  alpha        kinematic: link twist
%  jointtype    kinematic: 'R' if revolute, 'P' if prismatic
%  mdh          kinematic: 0 if standard D&H, else 1
%  offset       kinematic: joint variable offset
%  qlim         kinematic: joint variable limits [min max]
%-
%  m            dynamic: link mass
%  r            dynamic: link COG wrt link coordinate frame 3x1
%  I            dynamic: link inertia matrix, symmetric 3x3, about link COG.
%  B            dynamic: link viscous friction (motor referred)
%  Tc           dynamic: link Coulomb friction
%-
%  G            actuator: gear ratio
%  Jm           actuator: motor inertia (motor referred)
%
% Notes::
% - Methods inherited from the Link superclass.
% - This is reference class object
% - Link class objects can be used in vectors and arrays
%
% References::
% - Robotics, Vision & Control, P. Corke, Springer 2011, Chap 7.
%
% See also Link, Revolute, SerialLink.



% Copyright (C) 1993-2017, by Peter I. Corke
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
classdef Prismatic < Link
    methods
        function L = Prismatic(varargin)
            %Prismatic.Prismatic Create prismatic robot link object
            %
            % L = Prismatic(OPTIONS) is a prismatic link object with the kinematic and dynamic
            % parameters specified by the key/value pairs using the standard
            % Denavit-Hartenberg conventions.
            %
            % Options::
            % 'theta',TH    joint angle
            % 'a',A         joint offset (default 0)
            % 'alpha',A     joint twist (default 0)
            % 'standard'    defined using standard D&H parameters (default).
            % 'modified'    defined using modified D&H parameters.
            % 'offset',O    joint variable offset (default 0)
            % 'qlim',L      joint limit (default [])
            % 'I',I         link inertia matrix (3x1, 6x1 or 3x3)
            % 'r',R         link centre of gravity (3x1)
            % 'm',M         link mass (1x1)
            % 'G',G         motor gear ratio (default 1)
            % 'B',B         joint friction, motor referenced (default 0)
            % 'Jm',J        motor inertia, motor referenced (default 0)
            % 'Tc',T        Coulomb friction, motor referenced (1x1 or 2x1), (default [0 0])
            % 'sym'         consider all parameter values as symbolic not numeric
            %
            % Notes::
            % - The joint extension, d, is provided as an argument to the A() method.
            % - The link inertia matrix (3x3) is symmetric and can be specified by giving
            %   a 3x3 matrix, the diagonal elements [Ixx Iyy Izz], or the moments and products
            %   of inertia [Ixx Iyy Izz Ixy Iyz Ixz].
            % - All friction quantities are referenced to the motor not the load.
            % - Gear ratio is used only to convert motor referenced quantities such as
            %   friction and interia to the link frame.
            %
            % See also Link, Prismatic, RevoluteMDH.
            L = L@Link('prismatic', varargin{:});
            
            if nargin == 0
                L.d = [];
            end
            if isempty(L.theta)
                L.theta = 0;
            end
            assert(isempty(L.d), 'd cannot be specified for a prismatic link');
            L.jointtype = 'P';
        end
    end
end
