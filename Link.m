%LINK Robot manipulator Link class
%
% A Link object holds all information related to a robot link such as
% kinematics parameters, rigid-body inertial parameters, motor and
% transmission parameters.
%
% Methods::
%  A             link transform matrix
%  RP            joint type: 'R' or 'P'
%  friction      friction force
%  nofriction    Link object with friction parameters set to zero
%  dyn           display link dynamic parameters
%  islimit       test if joint exceeds soft limit
%  isrevolute    test if joint is revolute
%  isprismatic   test if joint is prismatic
%  display       print the link parameters in human readable form
%  char          convert to string
%
% Properties (read/write)::
%
%  theta    kinematic: joint angle
%  d        kinematic: link offset
%  a        kinematic: link length
%  alpha    kinematic: link twist
%  sigma    kinematic: 0 if revolute, 1 if prismatic
%  mdh      kinematic: 0 if standard D&H, else 1
%  offset   kinematic: joint variable offset
%  qlim     kinematic: joint variable limits [min max]
%-
%  m        dynamic: link mass
%  r        dynamic: link COG wrt link coordinate frame 3x1
%  I        dynamic: link inertia matrix, symmetric 3x3, about link COG.
%  B        dynamic: link viscous friction (motor referred)
%  Tc       dynamic: link Coulomb friction
%-
%  G        actuator: gear ratio
%  Jm       actuator: motor inertia (motor referred)
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
%
% References::
% - Robotics, Vision & Control, Chap 7,
%   P. Corke, Springer 2011.
%
% See also Link, Revolute, Prismatic, SerialLink, RevoluteMDH, PrismaticMDH.


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

classdef Link < handle
    properties
        % kinematic parameters
        theta % kinematic: link angle
        d     % kinematic: link offset
        alpha % kinematic: link twist
        a     % kinematic: link length
        sigma % revolute=0, prismatic=1
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
            %LINK Create robot link object
            %
            % This the class constructor which has several call signatures.
            %
            % L = Link() is a Link object with default parameters.
            %
            % L = Link(LNK) is a Link object that is a deep copy of the link
            % object LNK.
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
            % - It is an error to specify both 'theta' and 'd'
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
            %  - DH = [THETA D A ALPHA SIGMA OFFSET] where OFFSET is a constant displacement
            %    between the user joint angle vector and the true kinematic solution.
            %  - DH = [THETA D A ALPHA SIGMA] where SIGMA=0 for a revolute and 1 for a
            %    prismatic joint, OFFSET is zero.
            %  - DH = [THETA D A ALPHA], joint is assumed revolute and OFFSET is zero.
            %
            % Options::
            %
            % 'standard'    for standard D&H parameters (default).
            % 'modified'    for modified D&H parameters.
            % 'revolute'    for a revolute joint, can be abbreviated to 'r' (default)
            % 'prismatic'   for a prismatic joint, can be abbreviated to 'p'
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
            % - The parameter D is unused in a revolute joint, it is simply a placeholder
            %   in the vector and the value given is ignored.
            % - The parameter THETA is unused in a prismatic joint, it is simply a placeholder
            %   in the vector and the value given is ignored.
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
            % See also Revolute, Prismatic.
            
            %TODO eliminate legacy dyn matrix
            
            if nargin == 0
                % create an 'empty' Link object
                % this call signature is needed to support arrays of Links
                
                %% kinematic parameters
                l.alpha = 0;
                l.a = 0;
                l.theta = 0;
                l.d = 0;
                l.sigma = 0;
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
                l = copy(varargin{1});
                
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
                        l.sigma = 1;
                    end
                    
                    if ~isempty(opt.d)
                        % constant value of d means it must be revolute
                        l.d = value( opt.d, opt);
                        l.sigma = 0;
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
                        error('must provide params (theta d a alpha)');
                    end
                    
                    % set the kinematic parameters
                    l.theta = dh(1);
                    l.d = dh(2);
                    l.a = dh(3);
                    l.alpha = dh(4);
                    
                    l.sigma = 0;
                    l.offset = 0;
                    l.flip = false;
                    l.mdh = 0;  % default to standard D&H
                    
                    % optionally set sigma and offset
                    if length(dh) >= 5
                        l.sigma = dh(5);
                    end
                    if length(dh) == 6
                        l.offset = dh(6);
                    end
                    
                    if length(dh) > 6
                        % legacy DYN matrix
                        
                        l.sigma = dh(5);
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
            % F = L.friction(QD) is the joint friction force/torque for link velocity QD.
            %
            % Notes::
            % - The returned friction value is referred to the output of the gearbox.
            % - The friction parameters in the Link object are referred to the motor.
            % - Motor viscous friction is scaled up by G^2.
            % - Motor Coulomb friction is scaled up by G.
            % - The appropriate Coulomb friction value to use in the non-symmetric case
            %   depends on the sign of the joint velocity, not the motor velocity.
            
                
            tau = l.B * abs(l.G) * qd;
            
            if issym(l)
                tau = tau + l.Tc;
            elseif qd > 0
                tau = tau + l.Tc(1);
            elseif qd < 0
                tau = tau + l.Tc(2);
            end
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
            % See also SerialLink.nofriction, SerialLink.fdyn.
            
            l2 = Link(l);
            
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
            %Link.RP Joint type
            %
            % c = L.RP() is a character 'R' or 'P' depending on whether joint is revolute or
            % prismatic respectively.
            % If L is a vector of Link objects return a string of characters in joint order.
            v = '';
            for ll=l
                if ll.sigma == 0
                    v = strcat(v, 'R');
                else
                    v = strcat(v, 'P');
                end
            end
        end % RP()
        
        function set.r(l, v)
            %Link.r Set centre of gravity
            %
            % L.r = R sets the link centre of gravity (COG) to R (3-vector).
            %
            if isempty(v)
                return;
            end
            if length(v) ~= 3
                error('RTB:Link:badarg', 'COG must be a 3-vector');
            end
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
            
            if isa(v,'sym') && ~isempty(findsym(v))
                l.Tc = sym('Tc');
            elseif isa(v,'sym') && isempty(findsym(v))
                v = double(v);
            end
            
            if length(v) == 1  ~isa(v,'sym')
                l.Tc = [v -v]; 
            elseif length(v) == 2 && ~isa(v,'sym')
                if v(1) < v(2)
                    error('RTB:Link:badarg', 'Coulomb friction is [Tc+ Tc-]');
                end
                l.Tc = v;     
            else
                error('Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only')
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
                    error('inertia matrix must be symmetric');
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
            if isempty(l.qlim)
                error('no limits assigned to link')
            end
            v = (q > l.qlim(2)) - (q < l.qlim(1));
        end % islimit()
        
        function v = isrevolute(L)
            %Link.isrevolute  Test if joint is revolute
            %
            % L.isrevolute() is true (1) if joint is revolute.
            %
            % See also Link.isprismatic.
            
            v = [L.sigma] == 0;
        end
        
        function v = isprismatic(L)
            %Link.isprismatic  Test if joint is prismatic
            %
            % L.isprismatic() is true (1) if joint is prismatic.
            %
            % See also Link.isrevolute.
            v = [L.sigma] == 1;
        end
        
        
        function T = A(L, q)
            %Link.A Link transform matrix
            %
            % T = L.A(Q) is the link homogeneous transformation matrix (4x4) corresponding
            % to the link variable Q which is either the Denavit-Hartenberg parameter THETA (revolute)
            % or D (prismatic).
            %
            % Notes::
            % - For a revolute joint the THETA parameter of the link is ignored, and Q used instead.
            % - For a prismatic joint the D parameter of the link is ignored, and Q used instead.
            % - The link offset parameter is added to Q before computation of the transformation matrix.
            sa = sin(L.alpha); ca = cos(L.alpha);
            if L.flip
                q = -q + L.offset;
            else
                q = q + L.offset;
            end
            if L.sigma == 0
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
                
                conv = l.RP;
                if l.mdh == 0
                    conv = strcat(conv, ',stdDH');
                else
                    conv = strcat(conv, ',modDH');
                end
                if length(links) == 1
                    qname = 'q';
                else
                    qname = sprintf('q%d', j);
                end
                
                if from_robot
                    % invoked from SerialLink.char method, format for table
                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf('|%3d|%11s|%11s|%11s|%11s|%11s|', ...
                            j, ...
                            render(l.theta), ...
                            qname, ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset));
                    else
                        % revolute joint
                        js = sprintf('|%3d|%11s|%11s|%11s|%11s|%11s|', ...
                            j, ...
                            qname, ...
                            render(l.d), ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset));
                    end
                else
                    
                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf(' theta=%s, d=%s, a=%s, alpha=%s, offset=%s (%s)', ...
                            render(l.theta), ...
                            qname, ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset), ...
                            conv);
                    else
                        % revolute
                        js = sprintf(' theta=%s, d=%s, a=%s, alpha=%s, offset=%s (%s)', ...
                            qname, ...
                            render(l.d), ...
                            render(l.a), ...
                            render(l.alpha), ...
                            render(l.offset), ...
                            conv);
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
        function new = copy(this)
            
            for j=1:length(this)
                % Instantiate new object of the same class.
                %new(j) = feval(class(this(j)));
                new(j) = Link();
                % Copy all non-hidden properties.
                p = properties(this(j));
                for i = 1:length(p)
                    new(j).(p{i}) = this(j).(p{i});
                end
            end
        end
        
        function links = horzcat(this, varargin)
            links = this.toLink();
            
            for i=1:length(varargin)
                links = cat(2, links, varargin{i}.toLink());
            end
        end
        
        function links = vertcat(this, varargin)
            links = this.horzcat(varargin{:});
        end
        
        function new = toLink(this)
            for j=1:length(this)
                new(j) = Link();
                
                % Copy all non-hidden properties.
                p = properties(this(j));
                for i = 1:length(p)
                    new(j).(p{i}) = this(j).(p{i});
                end
            end
            
        end
        
        function res = issym(l)
            %Link.issym Check if link is a symbolic model
            %
            % res = L.issym() is true if the Link L has symbolic parameters.
 
            res = isa(l.alpha,'sym');
        end
        
        function sl = sym(l)
            
            sl = Link(l);   % clone the link
            
            if ~isempty(sl.theta)
                sl.theta = sym(sl.theta);
            end
            if ~isempty(sl.d)
                sl.d = sym(sl.d);
            end
            sl.alpha = sym(sl.alpha);
            sl.a = sym(sl.a);
            sl.offset = sym(sl.offset);
            
            sl.I = sym(sl.I);
            sl.r = sym(sl.r);
            sl.m = sym(sl.m);
            
            sl.Jm = sym(sl.Jm);
            sl.G = sym(sl.G);
            sl.B = sym(sl.B);
            sl.Tc = sym(sl.Tc);
            
        end
    end % methods
    

end % class

function s = render(v, fmt)
    if nargin < 2
        fmt = '%11.4g';
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
