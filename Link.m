%LinkRobot manipulator Link class
%
% A Link object holds all information related to a robot joint and link such as
% kinematics parameters, rigid-body inertial parameters, motor and
% transmission parameters.
%
% Constructors::
%  Link           general constructor
%  Prismatic      construct a prismatic joint+link using standard DH
%  PrismaticMDH   construct a prismatic joint+link using modified DH
%  Revolute       construct a revolute joint+link using standard DH
%  RevoluteMDH    construct a revolute joint+link using modified DH
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
% Examples::
%
%         L = Link([0 1.2 0.3 pi/2]);
%         L = Link('revolute', 'd', 1.2, 'a', 0.3, 'alpha', pi/2);
%         L = Revolute('d', 1.2, 'a', 0.3, 'alpha', pi/2);
%
% Notes::
% - This is a reference class object.
% - Link objects can be used in vectors and arrays.
% - Convenience subclasses are Revolute, Prismatic, RevoluteMDH and
%   PrismaticMDH.
%
% References::
% - Robotics, Vision & Control, P. Corke, Springer 2011, Chap 7.
%
% See also Link, Revolute, Prismatic, SerialLink, RevoluteMDH, PrismaticMDH.



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

classdef Link < matlab.mixin.Copyable
    properties
        % kinematic parameters
        theta % kinematic: link angle
        d     % kinematic: link offset
        alpha % kinematic: link twist
        a     % kinematic: link length
        jointtype  % revolute='R', prismatic='P' -- should be an enum
        mdh   % standard DH=0, MDH=1
        offset % joint coordinate offset
        name   % joint coordinate name
        flip   % joint moves in opposite direction
        
        % dynamic parameters
        m  % dynamic: link mass
        r  % dynamic: position of COM with respect to link frame (3x1)
        I  % dynamic: inertia of link with respect to COM (3x3)
        Jm % dynamic: motor inertia
        B  % dynamic: motor viscous friction (1x1 or 2x1)
        
        Tc % dynamic: motor Coulomb friction (1x2 or 2x1)
        G  % dynamic: gear ratio
        qlim % joint coordinate limits (2x1)
    end
    
    methods
        function l = Link(varargin)
            %Link Create robot link object
            %
            % This the class constructor which has several call signatures.
            %
            % L = Link() is a Link object with default parameters.
            %
            % L = Link(LNK) is a Link object that is a deep copy of the link
            % object LNK and has type Link, even if LNK is a subclass.
            %
            % L = Link(OPTIONS) is a link object with the kinematic and dynamic
            % parameters specified by the key/value pairs.
            %
            % Options::
            % 'theta',TH    joint angle, if not specified joint is revolute
            % 'd',D         joint extension, if not specified joint is prismatic
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
            % 'revolute'    for a revolute joint (default)
            % 'prismatic'   for a prismatic joint 'p'
            % 'standard'    for standard D&H parameters (default).
            % 'modified'    for modified D&H parameters.
            % 'sym'         consider all parameter values as symbolic not numeric
            %
            % Notes::
            % - It is an error to specify both 'theta' and 'd'
            % - The joint variable, either theta or d, is provided as an argument to
            %   the A() method.
            % - The link inertia matrix (3x3) is symmetric and can be specified by giving
            %   a 3x3 matrix, the diagonal elements [Ixx Iyy Izz], or the moments and products
            %   of inertia [Ixx Iyy Izz Ixy Iyz Ixz].
            % - All friction quantities are referenced to the motor not the load.
            % - Gear ratio is used only to convert motor referenced quantities such as
            %   friction and interia to the link frame.
            %
            % Old syntax::
            % L = Link(DH, OPTIONS) is a link object using the specified kinematic
            % convention  and with parameters:
            %  - DH = [THETA D A ALPHA SIGMA OFFSET] where SIGMA=0 for a revolute and 1
            %    for a prismatic joint; and OFFSET is a constant displacement between the
            %    user joint variable and the value used by the kinematic model.
            %  - DH = [THETA D A ALPHA SIGMA] where OFFSET is zero.
            %  - DH = [THETA D A ALPHA], joint is assumed revolute and OFFSET is zero.
            %
            % Options::
            %
            % 'standard'    for standard D&H parameters (default).
            % 'modified'    for modified D&H parameters.
            % 'revolute'    for a revolute joint, can be abbreviated to 'r' (default)
            % 'prismatic'   for a prismatic joint, can be abbreviated to 'p'
            %
            % Notes::
            % - The parameter D is unused in a revolute joint, it is simply a placeholder
            %   in the vector and the value given is ignored.
            % - The parameter THETA is unused in a prismatic joint, it is simply a placeholder
            %   in the vector and the value given is ignored.
            %
            % Examples::
            % A standard Denavit-Hartenberg link
            %        L3 = Link('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
            % since 'theta' is not specified the joint is assumed to be revolute, and
            % since the kinematic convention is not specified it is assumed 'standard'.
            %
            % Using the old syntax
            %        L3 = Link([ 0, 0.15005, 0.0203, -pi/2], 'standard');
            % the flag 'standard' is not strictly necessary but adds clarity.  Only 4 parameters
            % are specified so sigma is assumed to be zero, ie. the joint is revolute.
            %
            %        L3 = Link([ 0, 0.15005, 0.0203, -pi/2, 0], 'standard');
            % the flag 'standard' is not strictly necessary but adds clarity.  5 parameters
            % are specified and sigma is set to zero, ie. the joint is revolute.
            %
            %        L3 = Link([ 0, 0.15005, 0.0203, -pi/2, 1], 'standard');
            % the flag 'standard' is not strictly necessary but adds clarity.  5 parameters
            % are specified and sigma is set to one, ie. the joint is prismatic.
            %
            % For a modified Denavit-Hartenberg revolute joint
            %        L3 = Link([ 0, 0.15005, 0.0203, -pi/2, 0], 'modified');
            %
            % Notes::
            % - Link object is a reference object, a subclass of Handle object.
            % - Link objects can be used in vectors and arrays.
            % - The joint offset is a constant added to the joint angle variable before
            %   forward kinematics and subtracted after inverse kinematics.  It is useful
            %   if you  want the robot to adopt a 'sensible' pose for zero joint angle
            %   configuration.
            % - The link dynamic (inertial and motor) parameters are all set to
            %   zero.  These must be set by explicitly assigning the object
            %   properties: m, r, I, Jm, B, Tc.
            % - The gear ratio is set to 1 by default, meaning that motor friction and
            %   inertia will be considered if they are non-zero.
            %
            % See also Revolute, Prismatic, RevoluteMDH, PrismaticMDH.
            
            %TODO eliminate legacy dyn matrix
            
            if nargin == 0
                % create an 'empty' Link object
                % this call signature is needed to support arrays of Links
                
                %% kinematic parameters
                l.alpha = 0;
                l.a = 0;
                l.theta = 0;
                l.d = 0;
                l.jointtype = 'R';
                l.mdh = 0;
                l.offset = 0;
                l.flip = false;
                l.qlim = [];
                
                %% dynamic parameters
                % these parameters must be set by the user if dynamics is used
                l.m = 0;
                l.r = [0 0 0];
                l.I = zeros(3,3);
                
                % dynamic params with default (zero friction)
                l.Jm = 0;
                l.G = 1;
                l.B = 0;
                l.Tc = [0 0];
            elseif nargin == 1 && isa(varargin{1}, 'Link')
                % clone the passed Link object
                this = varargin{1};
                
                for j=1:length(this)
                    l(j) = Link();
                    
                    % Copy all non-hidden properties.
                    p = properties(this(j));
                    for i = 1:length(p)
                        l(j).(p{i}) = this(j).(p{i});
                    end
                end
                
            else
                % Create a new Link based on parameters
                
                % parse all possible options
                opt.theta = [];
                opt.a = 0;
                opt.d = [];
                opt.alpha = 0;
                opt.G = 0;
                opt.B = 0;
                opt.Tc = [0 0];
                opt.Jm = 0;
                opt.I = zeros(3,3);
                opt.m = 0;
                opt.r = [0 0 0];
                opt.offset = 0;
                opt.qlim = [];
                opt.type = {'revolute', 'prismatic', 'fixed'};
                opt.convention = {'standard', 'modified'};
                opt.sym = false;
                opt.flip = false;
                
                [opt,args] = tb_optparse(opt, varargin);
                
                % return a parameter as a number of symbol depending on
                % the 'sym' option
                
                if isempty(args)
                    % This is the new call format, where all parameters are
                    % given by key/value pairs
                    %
                    % eg. L3 = Link('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2);
                    if ~isempty(opt.theta)
                        % constant value of theta means it must be prismatic
                        l.theta = value( opt.theta, opt);
                        l.jointtype = 'P';
                    end
                    
                    if ~isempty(opt.d)
                        % constant value of d means it must be revolute
                        l.d = value( opt.d, opt);
                        l.jointtype = 'R';
                    end
                    if ~isempty(opt.d) && ~isempty(opt.theta)
                        error('RTB:Link:badarg', 'specify only one of ''d'' or ''theta''');
                    end
                    
                    l.a =     value( opt.a, opt);
                    l.alpha = value( opt.alpha, opt);
                    
                    l.offset = value( opt.offset, opt);
                    l.flip = value( opt.flip, opt);

                    l.qlim =   value( opt.qlim, opt);
                    
                    l.m = value( opt.m, opt);
                    l.r = value( opt.r, opt);
                    l.I = value( opt.I, opt);
                    l.Jm = value( opt.Jm, opt);
                    l.G = value( opt.G, opt);
                    l.B = value( opt.B, opt);
                    l.Tc = value( opt.Tc, opt);
                else
                    % This is the old call format, where all parameters are
                    % given by a vector containing kinematic-only, or
                    % kinematic plus dynamic parameters
                    %
                    % eg. L3 = Link([ 0, 0.15005, 0.0203, -pi/2, 0], 'standard');
                    dh = args{1};
                    if length(dh) < 4
                        error('RTB:Link:badarg', 'must provide params (theta d a alpha)');
                    end
                    
                    % set the kinematic parameters
                    l.theta = dh(1);
                    l.d = dh(2);
                    l.a = dh(3);
                    l.alpha = dh(4);
                    
                    l.jointtype = 'R';  % default to revolute
                    l.offset = 0;
                    l.flip = false;
                    l.mdh = 0;  % default to standard D&H
                    
                    % optionally set sigma and offset
                    if length(dh) >= 5
                        if dh(5) == 1
                            l.jointtype = 'P';
                        end
                    end
                    if length(dh) == 6
                        l.offset = dh(6);
                    end
                    
                    if length(dh) > 6
                        % legacy DYN matrix
                        
                        if dh(5) > 0
                            l.jointtype = 'P';
                        else
                            l.jointtype = 'R';
                        end
                        l.mdh = 0;  % default to standard D&H
                        l.offset = 0;
                        
                        % it's a legacy DYN matrix
                        l.m = dh(6);
                        l.r = dh(7:9).';     % a column vector
                        v = dh(10:15);
                        l.I = [ v(1) v(4) v(6)
                            v(4) v(2) v(5)
                            v(6) v(5) v(3)];
                        if length(dh) > 15
                            l.Jm = dh(16);
                        end
                        if length(dh) > 16
                            l.G = dh(17);
                        else
                            l.G = 1;
                        end
                        if length(dh) > 17
                            l.B = dh(18);
                        else
                            l.B = 0.0;
                        end
                        if length(dh) > 18
                            l.Tc = dh(19:20);
                        else
                            l.Tc = [0 0];
                        end
                        l.qlim = [];
                    else
                        % we know nothing about the dynamics
                        l.m = [];
                        l.r = [];
                        l.I = [];
                        l.Jm = [];
                        l.G = 0;
                        l.B = 0;
                        l.Tc = [0 0];
                        l.qlim = [];
                    end
                end
                
                % set the kinematic convention to be used
                if strcmp(opt.convention, 'modified')
                    l.mdh = 1;
                else
                    l.mdh = 0;
                end
                
            end
            
            function out = value(v, opt)
                if opt.sym
                    out = sym(v);
                else
                    out = v;
                end
            end
            
        end % link()
        
        function  tau = friction(l, qd)
            %Link.friction Joint friction force
            %
            % F = L.friction(QD) is the joint friction force/torque (1xN) for joint
            % velocity QD (1xN). The friction model includes:
            % - Viscous friction which is a linear function of velocity.
            % - Coulomb friction which is proportional to sign(QD).
            %
            % Notes::
            % - The friction value should be added to the motor output torque, it has a
            %   negative value when QD>0. 
            % - The returned friction value is referred to the output of the gearbox.
            % - The friction parameters in the Link object are referred to the motor.
            % - Motor viscous friction is scaled up by G^2.
            % - Motor Coulomb friction is scaled up by G.
            % - The appropriate Coulomb friction value to use in the non-symmetric case
            %   depends on the sign of the joint velocity, not the motor velocity.
            % - The absolute value of the gear ratio is used.  Negative gear ratios are
            %   tricky: the Puma560 has negative gear ratio for joints 1 and 3.
            %
            % See also Link.nofriction.
            
            % viscous friction   
            tau = l.B * abs(l.G) * qd;
            
            % Coulomb friction
            if ~isa(qd, 'sym')
                if qd > 0
                    tau = tau + l.Tc(1);
                elseif qd < 0
                    tau = tau + l.Tc(2);
                end
            end
            
            % scale up by gear ratio
            tau = -abs(l.G) * tau;     % friction opposes motion
        end % friction()
        
        function tau = friction2(l, qd)
            
                    % experimental code
            qdm = qd / l.G;
            taum = -l.B * qdm;
            if qdm  > 0
                taum = taum - l.Tc(1);
            elseif qdm < 0
                taum = taum - l.Tc(2);
            end
            tau = taum * l.G;
        end

        
        function  l2 = nofriction(l, only)
            %Link.nofriction Remove friction
            %
            % LN = L.nofriction() is a link object with the same parameters as L except
            % nonlinear (Coulomb) friction parameter is zero.
            %
            % LN = L.nofriction('all') as above except that viscous and Coulomb friction
            % are set to zero.
            %
            % LN = L.nofriction('coulomb') as above except that Coulomb friction is set to zero.
            %
            % LN = L.nofriction('viscous') as above except that viscous friction is set to zero.
            %
            % Notes::
            % - Forward dynamic simulation can be very slow with finite Coulomb friction.
            %
            % See also Link.friction, SerialLink.nofriction, SerialLink.fdyn.
            
            l2 = copy(l);
            
            if nargin == 1
                only = 'coulomb';
            end
            
            switch only
                case 'all'
                    l2.B = 0;
                    l2.Tc = [0 0];
                case 'viscous'
                    l2.B = 0;
                case 'coulomb'
                    l2.Tc = [0 0];
            end
        end
        
        function v = RP(l)
            warning('RTB:Link:deprecated', 'use the .type() method instead');
            v = l.type();
        end
        
        function v = type(l)
            %Link.type Joint type
            %
            % c = L.type() is a character 'R' or 'P' depending on whether joint is
            % revolute or prismatic respectively. If L is a vector of Link objects
            % return an array of characters in joint order.
            %
            % See also SerialLink.config.
            v = '';
            for ll=l
                switch ll.jointtype
                    case 'R'
                        v = strcat(v, 'R');
                    case 'P'
                        v = strcat(v, 'P');
                    otherwise
                        error('RTB:Link:badval', 'bad value for link jointtype %d', ll.type);
                end
            end
        end 
        
        function set.r(l, v)
            %Link.r Set centre of gravity
            %
            % L.r = R sets the link centre of gravity (COG) to R (3-vector).
            %
            if isempty(v)
                return;
            end
            assert(length(v) == 3, 'RTB:Link:badarg', 'COG must be a 3-vector');

            l.r = v(:).';
        end % set.r()
        
        function set.Tc(l, v)
            %Link.Tc Set Coulomb friction
            %
            % L.Tc = F sets Coulomb friction parameters to [F -F], for a symmetric
            % Coulomb friction model.
            %
            % L.Tc = [FP FM] sets Coulomb friction to [FP FM], for an asymmetric
            % Coulomb friction model. FP>0 and FM<0.  FP is applied for a positive
            % joint velocity and FM for a negative joint velocity.
            %
            % Notes::
            % - The friction parameters are defined as being positive for a positive
            %   joint velocity, the friction force computed by Link.friction uses the
            %   negative of the friction parameter, that is, the force opposing motion of
            %   the joint.
            %
            % See also Link.friction.
            if isempty(v)
                return;
            end
            
            if isa(v,'sym') && ~isempty(symvar(v))
                l.Tc = sym('Tc');
            elseif isa(v,'sym') && isempty(symvar(v))
                v = double(v);
            end
            
            if length(v) == 1  ~isa(v,'sym')
                l.Tc = [v -v]; 
            elseif length(v) == 2 && ~isa(v,'sym')
                assert(v(1) >= v(2), 'RTB:Link:badarg', 'Coulomb friction is [Tc+ Tc-]');
                l.Tc = v;     
            else
                error('RTB:Link:badarg', 'Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only')
            end
        end % set.Tc()
        
        function set.I(l, v)
            %Link.I Set link inertia
            %
            % L.I = [Ixx Iyy Izz] sets link inertia to a diagonal matrix.
            %
            % L.I = [Ixx Iyy Izz Ixy Iyz Ixz] sets link inertia to a symmetric matrix with
            % specified inertia and product of intertia elements.
            %
            % L.I = M set Link inertia matrix to M (3x3) which must be symmetric.
            if isempty(v)
                return;
            end
            if all(size(v) == [3 3])
                if isa(v, 'double') && norm(v-v') > eps
                    error('RTB:Link:badarg', 'inertia matrix must be symmetric');
                end
                l.I = v;
            elseif length(v) == 3
                l.I = diag(v);
            elseif length(v) == 6
                l.I = [ v(1) v(4) v(6)
                    v(4) v(2) v(5)
                    v(6) v(5) v(3) ];
            else
                error('RTB:Link:badarg', 'must set I to 3-vector, 6-vector or symmetric 3x3');
            end
        end % set.I()
        
        function v = islimit(l, q)
            %Link.islimit  Test joint limits
            %
            % L.islimit(Q) is true (1) if Q is outside the soft limits set for this joint.
            %
            % Note::
            % - The limits are not currently used by any Toolbox functions.
            assert(~isempty(l.qlim), 'RTB:Link:badarg', 'no limits assigned to link')
            v = (q > l.qlim(2)) - (q < l.qlim(1));
        end % islimit()
        
        function v = isrevolute(L)
            %Link.isrevolute  Test if joint is revolute
            %
            % L.isrevolute() is true (1) if joint is revolute.
            %
            % See also Link.isprismatic.
            
            v = [L.jointtype] == 'R';
        end
        
        function v = isprismatic(L)
            %Link.isprismatic  Test if joint is prismatic
            %
            % L.isprismatic() is true (1) if joint is prismatic.
            %
            % See also Link.isrevolute.
            v = ~L.isrevolute();
        end
        
        
        function T = A(L, q)
            %Link.A Link transform matrix
            %
            % T = L.A(Q) is an SE3 object representing the transformation between link
            % frames when the link variable Q which is either the Denavit-Hartenberg
            % parameter THETA (revolute) or D (prismatic).  For:
            %  - standard DH parameters, this is from the previous frame to the current.
            %  - modified DH parameters, this is from the current frame to the next.
            % 
            % Notes::
            % - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
            % - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
            % - The link offset parameter is added to Q before computation of the transformation matrix.
            %
            % See also SerialLink.fkine.
            sa = sin(L.alpha); ca = cos(L.alpha);
            if L.flip
                q = -q + L.offset;
            else
                q = q + L.offset;
            end
            if L.isrevolute
                % revolute
                st = sin(q); ct = cos(q);
                d = L.d;
            else
                % prismatic
                st = sin(L.theta); ct = cos(L.theta);
                d = q;
            end
            
            if L.mdh == 0
                % standard DH
                
                T =    [    ct  -st*ca  st*sa   L.a*ct
                    st  ct*ca   -ct*sa  L.a*st
                    0   sa      ca      d
                    0   0       0       1];
            else
                % modified DH
                
                T =    [    ct      -st     0   L.a
                    st*ca   ct*ca   -sa -sa*d
                    st*sa   ct*sa   ca  ca*d
                    0       0       0   1];
            end
            
            T = SE3(T);
            
        end % A()
        
        function display(l)
            %Link.display Display parameters
            %
            % L.display() displays the link parameters in compact single line format.  If L is a
            % vector of Link objects displays one line per element.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Link object and the command has no trailing
            %   semicolon.
            %
            % See also Link.char, Link.dyn, SerialLink.showlink.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(l) );
        end % display()
        
        function s = char(links, from_robot)
            %Link.char Convert to string
            %
            % s = L.char() is a string showing link parameters in a compact single line format.
            % If L is a vector of Link objects return a string with one line per Link.
            %
            % See also Link.display.
            
            % display in the order theta d a alpha
            if nargin < 2
                from_robot = false;
            end
            
            s = '';
            
            for j=1:length(links)
                l = links(j);
                
                if l.mdh == 0
                    conv = 'std';
                else
                    conv = 'mod';
                end
                if length(links) == 1
                    qname = 'q';
                else
                    qname = sprintf('q%d', j);
                end
                
                if from_robot
                    fmt = '%11g';
                    % invoked from SerialLink.char method, format for table
                    if l.isprismatic
                        % prismatic joint
                        js = sprintf('|%3d|%11s|%11s|%11s|%11s|%11s|', ...
                            j, ...
                            render(l.theta, fmt), ...
                            qname, ...
                            render(l.a, fmt), ...
                            render(l.alpha, fmt), ...
                            render(l.offset, fmt));
                    else
                        % revolute joint
                        js = sprintf('|%3d|%11s|%11s|%11s|%11s|%11s|', ...
                            j, ...
                            qname, ...
                            render(l.d, fmt), ...
                            render(l.a, fmt), ...
                            render(l.alpha, fmt), ...
                            render(l.offset, fmt));
                    end
                else
                    if length(links) == 1
                       if l.isprismatic
                        % prismatic joint
                        js = sprintf('Prismatic(%s): theta=%s, d=%s, a=%s, alpha=%s, offset=%s', ...
                            conv, ...
                            render(l.theta,'%g'), ...
                            qname, ...
                            render(l.a,'%g'), ...
                            render(l.alpha,'%g'), ...
                            render(l.offset,'%g') );
                    else
                        % revolute
                        js = sprintf('Revolute(%s): theta=%s, d=%s, a=%s, alpha=%s, offset=%s', ...
                            conv, ...
                            qname, ...
                            render(l.d,'%g'), ...
                            render(l.a,'%g'), ...
                            render(l.alpha,'%g'), ...
                            render(l.offset,'%g') );
                    end
                    else
                    if l.isprismatic
                        % prismatic joint
                        js = sprintf('Prismatic(%s): theta=%s   d=%s a=%s alpha=%s offset=%s', ...
                            conv, ...
                            render(l.theta), ...
                            qname, ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset) );
                    else
                        % revolute
                        js = sprintf('Revolute(%s):  theta=%s   d=%s a=%s alpha=%s offset=%s', ...
                            conv, ...
                            qname, ...
                            render(l.d), ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset) );
                    end
                    end
                end
                if isempty(s)
                    s = js;
                else
                    s = char(s, js);
                end
            end
            

        end % char()
        
        function dyn(links)
            %Link.dyn Show inertial properties of link
            %
            % L.dyn() displays the inertial properties of the link object in a multi-line
            % format. The properties shown are mass, centre of mass, inertia, friction,
            % gear ratio and motor properties.
            %
            % If L is a vector of Link objects show properties for each link.
            %
            % See also SerialLink.dyn.
            
            for j=1:numel(links)
                l = links(j);
                if numel(links) > 1
                    fprintf('\nLink %d::', j);
                end
                fprintf('%s\n', l.char());
                if ~isempty(l.m)
                    fprintf('  m    = %s\n', render(l.m))
                end
                if ~isempty(l.r)
                    s = render(l.r);
                    fprintf('  r    = %s %s %s\n', s{:});
                end
                if ~isempty(l.I)
                    s = render(l.I(1,:));
                    fprintf('  I    = | %s %s %s |\n', s{:});
                    s = render(l.I(2,:));
                    fprintf('         | %s %s %s |\n', s{:});
                    s = render(l.I(3,:));
                    fprintf('         | %s %s %s |\n', s{:});
                end
                if ~isempty(l.Jm)
                    fprintf('  Jm   = %s\n', render(l.Jm));
                end
                if ~isempty(l.B)
                    fprintf('  Bm   = %s\n', render(l.B));
                end
                if ~isempty(l.Tc)
                    fprintf('  Tc   = %s(+) %s(-)\n', ...
                        render(l.Tc(1)), render(l.Tc(2)));
                end
                if ~isempty(l.G)
                    fprintf('  G    = %s\n', render(l.G));
                end
                if ~isempty(l.qlim)
                    fprintf('  qlim = %f to %f\n', l.qlim(1), l.qlim(2));
                end
            end
        end % dyn()
        
        % Make a copy of a handle object.
        % http://www.mathworks.com/matlabcentral/newsreader/view_thread/257925
%         function new = copy(this)
%             
%             for j=1:length(this)
%                 % Instantiate new object of the same class.
%                 %new(j) = feval(class(this(j)));
%                 new(j) = Link();
%                 % Copy all non-hidden properties.
%                 p = properties(this(j));
%                 for i = 1:length(p)
%                     new(j).(p{i}) = this(j).(p{i});
%                 end
%             end
%         end
        
        function links = horzcat(varargin)
            %Link.horzcat  Concatenate link objects
            %
            % [L1 L2] is a vector that contains deep copies of the Link class objects
            % L1 and L2.  
            %
            % Notes::
            % - The elements of the vector are all of type Link.
            % - If the elements were of a subclass type they are convered to type Link.
            % - Extends to arbitrary number of objects in list.
            %
            % See also Link.plus.
            
            % convert all elements to Link type
            l = cellfun(@Link, varargin, 'UniformOutput', 0);
            
            % convert to vector, cell2mat won't do this for me
            links = cat(2, l{:});
        end
        
        function links = vertcat(this, varargin)
            links = this.horzcat(varargin{:});
        end
        
        
        function R = plus(L1, L2)
            %Link.plus  Concatenate link objects into a robot
            %
            % L1+L2 is a SerialLink object formed from deep copies of the Link class objects
            % L1 and L2.  
            %
            % Notes::
            % - The elements can belong to any of the Link subclasses.
            % - Extends to arbitrary number of objects, eg. L1+L2+L3+L4.
            %
            % See also SerialLink, SerialLink.plus, Link.horzcat.
            assert( isa(L1, 'Link') && isa(L2, 'Link'), 'RTB:Link: second operand for + operator must be a Link class');
            
            R = SerialLink([L1 L2]);
            
        end
        
        function res = issym(l)
            %Link.issym Check if link is a symbolic model
            %
            % res = L.issym() is true if the Link L has any symbolic parameters.
            %
            % See also Link.sym.
 
            res = any( cellfun(@(x) isa(l.(x), 'sym'), properties(l)) );
        end
        
        function l = sym(l)
            %Link.sym Convert link parameters to symbolic type
            %
            % LS = L.sym is a Link object in which all the parameters are symbolic
            % ('sym') type.
            %
            % See also Link.issym.
            
%             sl = Link(l);   % clone the link
            
            if ~isempty(l.theta)
                l.theta = sym(l.theta);
            end
            if ~isempty(l.d)
                l.d = sym(l.d);
            end
            l.alpha = sym(l.alpha);
            l.a = sym(l.a);
            l.offset = sym(l.offset);
            
            l.I = sym(l.I);
            l.r = sym(l.r);
            l.m = sym(l.m);
            
            l.Jm = sym(l.Jm);
            l.G = sym(l.G);
            l.B = sym(l.B);
            l.Tc = sym(l.Tc);
        end
        

    end % methods
    

end % class

function s = render(v, fmt)
    if nargin < 2
        fmt = '%-11.4g';
    end
    if length(v) == 1
                if isa(v, 'double')
            s = sprintf(fmt, v);
        elseif isa(v, 'sym')
            s = char(v);
        else
            error('RTB:Link:badarg', 'Link parameter must be numeric or symbolic');
        end
    else
        
    for i=1:length(v)
        if isa(v, 'double')
            s{i} = sprintf(fmt, v(i));
        elseif isa(v, 'sym')
            s{i} = char(v(i));
        else
            error('RTB:Link:badarg', 'Link parameter must be numeric or symbolic');
        end
    end
    end
end
