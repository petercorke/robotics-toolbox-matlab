%LINK Robot manipulator Link class
%
% A Link object holds all information related to a robot link such as
% kinematics parameteres, rigid-body inertial parameters, motor and
% transmission parameters.
%
% L = Link([theta d a alpha]) is a link object with the specified kinematic 
% parameters theta, d, a and alpha.
%
% Methods::
%  A             return link transform (A) matrix
%  RP            return joint type: 'R' or 'P'
%  friction      return friction force
%  nofriction    return Link object with friction parameters set to zero
%  dyn           display link dynamic parameters
%  islimit       true if joint exceeds soft limit
%  isrevolute    true if joint is revolute
%  isprismatic   true if joint is prismatic
%  nofriction    remove joint friction
%  display       print the link parameters in human readable form
%  char          convert the link parameters to human readable string
%
% Properties (read/write)::
%
%  alpha   kinematic: link twist
%  a       kinematic: link twist
%  theta   kinematic: link twist
%  d       kinematic: link twist
%  sigma   kinematic: 0 if revolute, 1 if prismatic
%  mdh     kinematic: 0 if standard D&H, else 1
%  offset  kinematic: joint variable offset
%  qlim    kinematic: joint variable limits [min max]
%
%  m       dynamic: link mass
%  r       dynamic: link COG wrt link coordinate frame 3x1
%  I       dynamic: link inertia matrix, symmetric 3x3, about link COG.
%  B       dynamic: link viscous friction (motor referred)
%  Tc      dynamic: link Coulomb friction
%
%  G       actuator: gear ratio
%  Jm      actuator: motor inertia (motor referred)
%
% Notes::
% - this is reference class object
% - Link objects can be used in vectors and arrays
%
% See also SerialLink, Link.Link.

% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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
        function l = Link(dh, convention)
        %LINK Create robot link object
        %
        % This is class constructor function which has several call signatures.
        %
        % L = Link() is a Link object with default parameters.
        %
        % L = Link(L1) is a Link object that is a deep copy of the object L1.
        %
        % L = Link(DH, options) is a link object formed from the kinematic 
        % parameter vector:
        %  - DH = [theta d a alpha sigma offset] where offset is a constant added to 
        %    the joint angle variable before forward kinematics and is useful if you 
        %    want the robot to adopt a 'sensible' pose for zero joint angle 
        %    configuration.
        %  - DH = [theta d a alpha sigma] where sigma=0 for a revolute and 1 for a
        %    prismatic joint, offset is zero.
        %  - DH = [theta d a alpha], joint is assumed revolute and offset is zero.
        %
        % Options::
        %
        % 'standard'   for standard D&H parameters (default).
        % 'modified'   for modified D&H parameters.
        %
        % Notes:
        % - Link object is a reference object, a subclass of Handle object.
        % - Link objects can be used in vectors and arrays
        % - the parameter theta or d is unused in a revolute or prismatic 
        %   joint respectively, it is simply a placeholder for the joint 
        %   variable passed to L.A()
        % - the link dynamic (inertial and motor) parameters are all set to
        %   zero.  These must be set by explicitly assigning the object
        %   properties: m, r, I, Jm, B, Tc, G.

 %TODO eliminate legacy dyn matrix
 
            if nargin == 0
                % create an 'empty' link
                l.alpha = 0;
                l.a = 0;
                l.theta = 0;
                l.d = 0;
                l.sigma = 0;
                l.mdh = 0;
                l.offset = 0;
                
                l.m = [];
                l.r = [];
                v = [];
                l.I = [];
                l.Jm = [];
                l.G = [];
                l.B = 0;
                l.Tc = [0 0];
                l.qlim = [];

            elseif isa(dh, 'Link')
                % clone passed link
                l = copy(dh);

            else
                % legacy DH matrix
                % link([theta d a alpha])
                % link([theta d a alpha sigma])
                % link([theta d a alpha sigma offset])

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
                l.mdh = 0;  % default to standard D&H

                % optionally set sigma and offset
                if length(dh) >= 5
                    l.sigma = dh(5);
                end
                if length(dh) == 6
                    l.offset = dh(6);
                end

                % set the kinematic convention to be used
                if nargin > 1
                    if strncmp(convention, 'mod', 3) == 1
                        l.mdh = 1;
                    elseif strncmp(convention, 'sta', 3) == 1
                        l.mdh = 0;
                    else
                        error('convention must be modified or standard');
                    end
                end

                if length(dh) > 6
                    % legacy DYN matrix

                    l.sigma = dh(5);
                    l.mdh = 0;	% default to standard D&H
                    l.offset = 0;
                    
                    % it's a legacy DYN matrix
                    l.m = dh(6);
                    l.r = dh(7:9)';		% a column vector
                    v = dh(10:15);
                    l.I = [	v(1) v(4) v(6)
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
                    v = [];
                    l.I = [];
                    l.Jm = [];
                    l.G = [];
                    l.B = 0;
                    l.Tc = [0 0];
                    l.qlim = [];
                end
            end
        end % link()

        function  tau = friction(l, qd)
        %Link.friction Joint friction force
        %
        % F = L.friction(QD) is the joint friction force/torque for link velocity QD
            tau = 0.0;

            tau = l.B * qd;

            if qd > 0
                tau = tau + l.Tc(1);
            elseif qd < 0
                tau = tau + l.Tc(2);
            end
            tau = -tau;     % friction opposes motion
        end % friction()

        function  l2 = nofriction(l, only)
        %Link.nofriction Remove friction
        %
        % LN = L.nofriction() is a link object with the same parameters as L except 
        % nonlinear (Coulomb) friction parameter is zero.
        %
        % LN = L.nofriction('all') is a link object with the same parameters as L 
        % except all friction parameters are zero.

            l2 = Link(l);
            if (nargin == 2) && strcmpi(only(1:3), 'all')
                l2.B = 0;
            end
            l2.Tc = [0 0];
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

        function l = set.r(l, v)
        %Link.r Set centre of gravity
        %
        % L.r = r set the link centre of gravity (COG) to the 3-vector r.
        %
            if isempty(v)
                return;
            end
            if length(v) ~= 3
                error('COG must be a 3-vector');
            end
            l.r = v(:)';
        end % set.r()

        function l = set.Tc(l, v)
        %Link.Tc Set Coulomb friction
        %
        % L.Tc = F set Coulomb friction parameters to [FP FM], for a symmetric
        % Coulomb friction model.
        %
        % L.Tc = [FP FM] set Coulomb friction to [FP FM], for an asymmetric
        % Coulomb friction model. FP>0 and FM<0.
        %
        % See also Link.friction.
            if isempty(v)
                return;
            end
            if length(v) == 1
                l.Tc = [v -v];
            elseif length(v) == 2
                if v(1) < v(2)
                    error('Coulomb friction is [Tc+ Tc-]');
                end
                l.Tc = v;
            else
                error('Coulomb friction vector can have 1 (symmetric) or 2 (asymmetric) elements only')
            end
        end % set.Tc()

        function l = set.I(l, v)
        %Link.I Set link inertia
        %
        % L.I = [Ixx Iyy Izz] set Link inertia to a diagonal matrix.
        %
        % L.I = [Ixx Iyy Izz Ixy Iyz Ixz] set Link inertia to a symmetric matrix with
        % specified intertia and product of intertia elements.
        %
        % L.I = M set Link inertia matrix to 3x3 matrix M (which must be symmetric).
            if isempty(v)
                return;
            end
            if all(size(v) == [3 3])
                if norm(v-v') > eps
                    error('inertia matrix must be symmetric');
                end
                l.I = v;
            elseif length(v) == 3
                l.I = diag(v);
            elseif length(v) == 6
                l.I = [ v(1) v(4) v(6)
                    v(4) v(2) v(5)
                    v(6) v(5) v(3)];
            end
        end % set.I()

        function v = islimit(l, q)
        %Link.islimit  Test joint limits
        %
        % L.islimit(Q) is true (1) if Q is outside the soft limits set for this joint.
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
            
            v = L.sigma == 0;
        end

        function v = isprismatic(L)
        %Link.isprismatic  Test if joint is prismatic
        %
        % L.isprismatic() is true (1) if joint is prismatic.
        %
        % See also Link.isrevolute.
            v = L.sigma == 1;
        end
        

        function T = A(L, q)
        %Link.A Link transform matrix
        %
        % T = L.A(Q) is the 4x4 link homogeneous transformation matrix corresponding
        % to the link variable Q which is either theta (revolute) or d (prismatic).
        %
        % Notes::
        % - For a revolute joint the theta parameter of the link is ignored, and Q used instead.
        % - For a prismatic joint the d parameter of the link is ignored, and Q used instead.
        % - The link offset parameter is added to Q before computation of the transformation matrix.
            if L.mdh == 0
                T = linktran([L.alpha L.a L.theta L.d L.sigma], ...
                    q+L.offset);
            else
                T = mlinktran([L.alpha L.a L.theta L.d L.sigma], ...
                    q+L.offset);
            end
        end % A()

        function display(l)
        %Link.display Display parameters
        %
        % L.display() display link parameters in compact single line format.  If L is a 
        % vector of Link objects display one line per element.
        %
        % Notes::
        % - this method is invoked implicitly at the command line when the result
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
        %Link.char String representation of parameters
        %
        % s = L.char() is a string showing link parameters in compact single line format.  
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
                if from_robot
                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf('|%3d|%11.4g|%11s|%11.4g|%11.4g|', ...
                            j, l.theta, sprintf('q%d', j), l.a, l.alpha);
                    else
                        js = sprintf('|%3d|%11s|%11.4g|%11.4g|%11.4g|', ...
                            j, sprintf('q%d', j), l.d, l.a, l.alpha);
                    end
                else
                    conv = l.RP;
                    if l.mdh == 0
                        conv = [conv ',stdDH'];
                    else
                        conv = [conv ',modDH'];
                    end
                    if length(links) == 1
                        qname = 'q';
                    else
                        qname = sprintf('q%d', j);
                    end

                    if l.sigma == 1
                        % prismatic joint
                        js = sprintf(' theta=%.4g, d=%s, a=%.4g, alpha=%.4g (%s)', ...
                            l.theta, qname, l.a, l.alpha, conv);
                    else
                        js = sprintf(' theta=%s, d=%.4g, a=%.4g, alpha=%.4g (%s)', ...
                            qname, l.d, l.a, l.alpha, conv);
                    end
                end
                s = strvcat(s, js);
            end
        end % char()

        function dyn(l)
        %Link.dyn Display the inertial properties of link
        %
        % L.dyn() displays the inertial properties of the link object in a multi-line format.
        % The properties shown are mass, centre of mass, inertia, friction, gear ratio
        % and motor properties.
        %
        % If L is a vector of Link objects show properties for each element.

            if length(l) > 1
                for j=1:length(l)
                    ll = l(j);
                    fprintf('%d: %f; %f %f %f; %f %f %f\n', ...
                        j, ll.m, ll.r, diag(ll.I));
                    %dyn(ll);
                end
                return;
            end

            display(l);
            if ~isempty(l.m)
                fprintf('  m    = %f\n', l.m)
            end
            if ~isempty(l.r)
                fprintf('  r    = %f %f %f\n', l.r);
            end
            if ~isempty(l.I)
                fprintf('  I    = | %f %f %f |\n', l.I(1,:));
                fprintf('         | %f %f %f |\n', l.I(2,:));
                fprintf('         | %f %f %f |\n', l.I(3,:));
            end
            if ~isempty(l.Jm)
                fprintf('  Jm   = %f\n', l.Jm);
            end
            if ~isempty(l.B)
                fprintf('  Bm   = %f\n', l.B);
            end
            if ~isempty(l.Tc)
                fprintf('  Tc   = %f(+) %f(-)\n', l.Tc(1), l.Tc(2));
            end
            if ~isempty(l.G)
                fprintf('  G    = %f\n', l.G);
            end
            if ~isempty(l.qlim)
                fprintf('  qlim = %f to %f\n', l.qlim(1), l.qlim(2));
            end
        end % show()

        % Make a copy of a handle object. 
        % http://www.mathworks.com/matlabcentral/newsreader/view_thread/257925
        function new = copy(this) 

            for j=1:length(this)
                % Instantiate new object of the same class. 
                new(j) = feval(class(this(j))); 
                % Copy all non-hidden properties. 
                p = properties(this(j)); 
                for i = 1:length(p) 
                    new(j).(p{i}) = this(j).(p{i}); 
                end 
            end
        end 

        end % methods
    end % class

%%%%%%%%%%%% static class methods

%LINKTRAN   Compute the link transform from kinematic parameters
%
%   LINKTRAN(alpha, an, theta, dn)
%   LINKTRAN(DH, q) is a homogeneous 
%   transformation between link coordinate frames.
%
%   alpha is the link twist angle
%   an is the link length
%   theta is the link rotation angle
%   dn is the link offset
%   sigma is 0 for a revolute joint, non-zero for prismatic
%
%   In the second case, q is substitued for theta or dn according to sigma.
%
%   Based on the standard Denavit and Hartenberg notation.

function t = linktran(a, b, c, d)

    if nargin == 4
        alpha = a;
        an = b;
        theta = c;
        dn = d;
    else
        if numcols(a) < 4
            error('too few columns in DH matrix');
        end
        alpha = a(1);
        an = a(2);
        if numcols(a) > 4
            if a(5) == 0,   % revolute
                theta = b;
                dn = a(4);
            else        % prismatic
                theta = a(3);
                dn = b;
            end
        else
            theta = b;  % assume revolute if sigma not given
            dn = a(4);
        end
    end
    sa = sin(alpha); ca = cos(alpha);
    st = sin(theta); ct = cos(theta);

    t =    [    ct  -st*ca  st*sa   an*ct
            st  ct*ca   -ct*sa  an*st
            0   sa  ca  dn
            0   0   0   1];
end

%MLINKTRAN  Compute the link transform from kinematic parameters
%
%   MLINKTRAN(alpha, an, theta, dn)
%   MLINKTRAN(DH, q) is a homogeneous 
%   transformation between link coordinate frames.
%
%   alpha is the link twist angle
%   an is the link length
%   theta is the link rotation angle
%   dn is the link offset
%   sigma is 0 for a revolute joint, non-zero for prismatic
%
%   In the second case, q is substitued for theta or dn according to sigma.
%
%   Based on the modified Denavit and Hartenberg notation.

function t = mlinktran(a, b, c, d)

    if nargin == 4
        alpha = a;
        an = b;
        theta = c;
        dn = d;
    else
        if numcols(a) < 4
            error('too few columns in DH matrix');
        end
        alpha = a(1);
        an = a(2);
        if numcols(a) > 4
            if a(5) == 0,   % revolute
                theta = b;
                dn = a(4);
            else        % prismatic
                theta = a(3);
                dn = b;
            end
        else
            theta = b;  % assume revolute if no sigma given
            dn = a(4);
        end
    end
    sa = sin(alpha); ca = cos(alpha);
    st = sin(theta); ct = cos(theta);

    t =    [    ct  -st 0   an
            st*ca   ct*ca   -sa -sa*dn
            st*sa   ct*sa   ca  ca*dn
            0   0   0   1];
end

