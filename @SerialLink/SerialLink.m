%SerialLink Serial-link robot class
%
% A concrete class that represents a serial-link arm-type robot.  Each link
% and joint in the chain is described by a Link-class object using Denavit-Hartenberg
% parameters (standard or modified).
%
% Constructor methods::
%  SerialLink    general constructor
%  L1+L2         construct from Link objects
%
% Display/plot methods::
%  animate       animate robot model
%  display       print the link parameters in human readable form
%  dyn           display link dynamic parameters
%  edit          display and edit kinematic and dynamic parameters
%  getpos        get position of graphical robot
%  plot          display graphical representation of robot
%  plot3d        display 3D graphical model of robot
%  teach         drive the graphical robot
%
% Testing methods::
%  islimit       test if robot at joint limit
%  isconfig      test robot joint configuration
%  issym         test if robot has symbolic parameters
%  isprismatic   index of prismatic joints
%  isrevolute    index of revolute joints
%  isspherical   test if robot has spherical wrist
%  isdh          test if robot has standard DH model
%  ismdh         test if robot has modified DH model
%
% Conversion methods::
%  char          convert to string
%  sym           convert to symbolic parameters
%  todegrees     convert joint angles to degrees
%  toradians     convert joint angles to radians

% Kinematic methods::
%  A             link transforms
%  fkine         forward kinematics
%  ikine6s       inverse kinematics for 6-axis spherical wrist revolute robot
%  ikine         inverse kinematics using iterative numerical method
%  ikunc         inverse kinematics using optimisation
%  ikcon         inverse kinematics using optimisation with joint limits
%  ikine_sym     analytic inverse kinematics obtained symbolically
%  trchain       forward kinematics as a chain of elementary transforms
%  DH            convert modified DH model to standard
%  MDH           convert standard DH model to modified
%  twists        joint axis twists                       
%
% Velocity kinematic methods::
%  fellipse      display force ellipsoid
%  jacob0        Jacobian matrix in world frame
%  jacobe        Jacobian matrix in end-effector frame
%  jacob_dot     Jacobian derivative
%  maniplty      manipulability
%  vellipse      display velocity ellipsoid
%  qmincon       null space motion to centre joints between limits
%
% Dynamics methods::
%  accel           joint acceleration
%  cinertia        Cartesian inertia matrix
%  coriolis        Coriolis joint force
%  fdyn            forward dynamics
%  friction        friction force
%  gravjac         gravity load and Jacobian
%  gravload        gravity joint force
%  inertia         joint inertia matrix
%  itorque         inertial torque
%  jointdynamics   model of LTI joint dynamics
%  nofriction      set friction parameters to zero
%  perturb         randomly perturb link dynamic parameters
%  rne             inverse dynamics
%-
%  pay             payload effect
%  payload         add a payload in end-effector frame
%  gravjac         gravity load and Jacobian
%  paycap          payload capacity
%
% Operation methods::
%  jtraj         joint space trajectory
%  gencoords     symbolic generalized coordinates
%  genforces     symbolic generalized forces
%
% Properties (read/write)::
%  links      vector of Link objects (1xN)
%  gravity    direction of gravity [gx gy gz]
%  base       pose of robot's base (4x4 homog xform)
%  tool       robot's tool transform, T6 to tool tip (4x4 homog xform)
%  qlim       joint limits, [qmin qmax] (Nx2)
%  offset     kinematic joint coordinate offsets (Nx1)
%  name       name of robot, used for graphical display
%  manuf      annotation, manufacturer's name
%  comment    annotation, general comment
%  plotopt    options for plot() method (cell array)
%  fast       use MEX version of RNE.  Can only be set true if the mex
%             file exists.  Default is true.
%
% Properties (read only)::
%  n           number of joints
%  config      joint configuration string, eg. 'RRRRRR'
%  mdh         kinematic convention boolean (0=DH, 1=MDH)
%  theta       kinematic: joint angles (1xN)
%  d           kinematic: link offsets (1xN)
%  a           kinematic: link lengths (1xN)
%  alpha       kinematic: link twists (1xN)
%
% Overloaded operators::
%  R+L     append a Link object to a SerialLink manipulator
%  R1*R2   concatenate two SerialLink manipulators R1 and R2
%
% Note::
%  - SerialLink is a reference object.
%  - SerialLink objects can be used in vectors and arrays
%
% Reference::
% - Robotics, Vision & Control, Chaps 7-9,
%   P. Corke, Springer 2011.
% - Robot, Modeling & Control,
%   M.Spong, S. Hutchinson & M. Vidyasagar, Wiley 2006.
%
% See also Link, DHFactor.



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

% TODO:
% fix payload as per GG discussion
% if we subclass mixin.Copyable there are problem with
                        %subclass constructures...

classdef SerialLink < handle & dynamicprops % & matlab.mixin.Copyable

    properties
        name
        gravity
        base
        tool
        manufacturer
        comment

        plotopt  % options for the plot method, follow plot syntax
        
        fast    % mex version of rne detected
        
        interface   % interface to a real robot platform
        
        ikineType
        
        trail   % to support trail option
        
        % to support plot method
        movie
        delay
        loop
        
        % to support plot3d method
        model3d
        faces
        points
        plotopt3d % options for the plot3d method, follow plot3d syntax

    end

    events
        Moved
    end

    properties (SetAccess = private)
        % needs to be public readable for access by MEX file
        n
        links
        T
        mdh
    end
    

    properties (Dependent = true, SetAccess = private)
        config
    end

    properties (Dependent = true)
        offset
        qlim
        d
        a
        alpha
        theta
    end

    methods
        function r = SerialLink(varargin)
        %SerialLink Create a SerialLink robot object
        %
        % R = SerialLink(LINKS, OPTIONS) is a robot object defined by a vector 
        % of Link class objects which includes the subclasses Revolute,
        % Prismatic, RevoluteMDH or PrismaticMDH.
        %
        % R = SerialLink(OPTIONS) is a null robot object with no links.
        %
        % R = SerialLink([R1 R2 ...], OPTIONS) concatenate robots, the base of
        % R2 is attached to the tip of R1.  Can also be written as R1*R2 etc.
        %
        % R = SerialLink(R1, options) is a deep copy of the robot object R1, 
        % with all the same properties.
        %
        % R = SerialLink(DH, OPTIONS) is a robot object with kinematics defined by
        % the matrix DH which has one row per joint and each row is [theta d a
        % alpha] and joints are assumed revolute.  An optional fifth column sigma
        % indicate revolute (sigma=0) or prismatic (sigma=1).  An optional sixth
        % column is the joint offset.
        %
        % Options::
        %
        %  'name',NAME             set robot name property to NAME
        %  'comment',COMMENT       set robot comment property to COMMENT
        %  'manufacturer',MANUF    set robot manufacturer property to MANUF
        %  'base',T                set base transformation matrix property to T
        %  'tool',T                set tool transformation matrix property to T
        %  'gravity',G             set gravity vector property to G
        %  'plotopt',P             set default options for .plot() to P
        %  'plotopt3d',P           set default options for .plot3d() to P
        %  'nofast'                don't use RNE MEX file
        %  'configs',P             provide a cell array of predefined
        %                          configurations, as name, value pairs
        %
        % Examples::
        %
        % Create a 2-link robot
        %        L(1) = Link([ 0     0   a1  pi/2], 'standard');
        %        L(2) = Link([ 0     0   a2  0], 'standard');
        %        twolink = SerialLink(L, 'name', 'two link');
        %
        % Create a 2-link robot (most descriptive)
        %        L(1) = Revolute('d', 0, 'a', a1, 'alpha', pi/2);
        %        L(2) = Revolute('d', 0, 'a', a2, 'alpha', 0);
        %        twolink = SerialLink(L, 'name', 'two link');
        %
        % Create a 2-link robot (least descriptive)
        %        twolink = SerialLink([0 0 a1 0; 0 0 a2 0], 'name', 'two link');
        %
        % Robot objects can be concatenated in two ways
        %        R = R1 * R2;
        %        R = SerialLink([R1 R2]);
        %
        % Note::
        % - SerialLink is a reference object, a subclass of Handle object.
        % - SerialLink objects can be used in vectors and arrays
        % - Link subclass elements passed in must be all standard, or all modified,
        %   DH parameters.
        % - When robots are concatenated (either syntax) the intermediate base and
        %   tool transforms are removed since general constant transforms cannot 
        %   be represented in Denavit-Hartenberg notation.
        %
        % See also Link, Revolute, Prismatic, RevoluteMDH, PrismaticMDH, SerialLink.plot.


            r.links = [];
            r.n = 0;
            
            % default properties
            default.name = 'noname';
            default.comment = '';
            default.manufacturer = '';
            default.base = eye(4,4);
            default.tool = eye(4,4);
            default.gravity = [0; 0; 9.81];
            
            
            % process the rest of the arguments in key, value pairs
            opt.name = [];
            opt.comment = [];
            opt.manufacturer = [];
            opt.base = [];
            opt.tool = [];
            opt.gravity = [];
            
            opt.offset = [];
            opt.qlim = [];
            opt.plotopt = {};
            opt.plotopt3d = {};
            opt.ikine = [];
            opt.fast = r.fast;
            opt.modified = false;
            opt.configs = [];

            [opt,arg] = tb_optparse(opt, varargin);
            

            if length(arg) == 1
                % at least one argument, either a robot or link array
                
                L = arg{1};
                
                if isa(L, 'Link')
                    % passed an array of Link objects
                    r.links = L;
                    
                elseif numcols(L) >= 4 && (isa(L, 'double') || isa(L, 'sym'))
                    % passed a legacy DH matrix
                    dh_dyn = L;
                    clear L
                    for j=1:numrows(dh_dyn)
                        if opt.modified
                            L(j) =        Link(dh_dyn(j,:), 'modified');
                        else
                            L(j) =        Link(dh_dyn(j,:));
                        end                      
                    end
                    r.links = L;
                    
                elseif isa(L, 'SerialLink')
                    % passed a SerialLink object
                    if length(L) == 1
                        % clone the passed robot and the attached links
                        copy(r, L); 

                        r.links = copy(L.links);
                        default.name = [L.name ' copy'];
                    else
                        % compound the robots in the vector
                        copy(r, L(1));
                        
                        for k=2:length(L)
                            r.links = [r.links copy(L(k).links)];
                            r.name = [r.name '+' L(k).name];
                        end
                        
                        r.tool = L(end).tool; % tool of composite robot from the last one
                        r.gravity = L(1).gravity; % gravity of composite robot from the first one
                    end
                    
                else
                    error('unknown type passed to robot');
                end
                r.n = length(r.links);
            end
            
            
            % set properties of robot object from passed options or defaults
            for p=properties(r)'
                p = p{1};  % property name
                
                if isfield(opt, p) && ~isempty( opt.(p) )
                    % if there's a set option, override what's in the robot
                    r.(p) = opt.(p);
                end
                
                if isfield(default, p) && isempty( r.(p) ) 
                    % otherwise if there's a set default, use that
                    r.(p) = default.(p);
                end
            end
            
            % other properties

            if exist('frne') == 3
                r.fast = true;
            else
                r.fast = false;
            end
            
            r.ikineType = opt.ikine;
            
            r.gravity = r.gravity(:);
            assert(length(r.gravity) == 3, 'RTB:SerialLink:badarg', 'gravity must be a 3-vector');

            % set the robot object mdh status flag
            if ~isempty(r.links)
                mdh = [r.links.mdh];
                if all(mdh == 0)
                    r.mdh = mdh(1);
                elseif all (mdh == 1)
                    r.mdh = mdh(1);
                else
                    error('RTB:SerialLink:badarg', 'robot has mixed D&H links conventions');
                end
            end
            
            if ~isempty(opt.configs)
                % add passed configurations as dynamic properties of the robot
                for i=1:2:length(opt.configs)
                    name = opt.configs{i}; val = opt.configs{i+1};
                    assert(ischar(name), 'configs: expecting a char array')
                    assert(isnumeric(val) && length(val) == length(r.links), 'configs: expecting a n-vector');
                    addprop(r, name);
                    r.(name) = val;
                end
            end
            
        end

        %{
        function delete(r)
            disp('in destructor');
            if ~isempty(r.teachfig)
                delete(r.teachfig);
            end
            rh = findobj('Tag', r.name);
            for f=rh
                delete(f);
            end
        end
        %}
        
        function copy(out, in)
            C = metaclass(in);
            P = C.Properties;
            
            for k=1:length(P)
                if ~P{k}.Dependent
                    out.(P{k}.Name) = in.(P{k}.Name);
                end
            end
        end
        
        function r2 = plus(R, L)
            %SerialLink.plus  Append a link objects to a robot
            %
            % R+L is a SerialLink object formed appending a deep copy of the Link L to the SerialLink
            % robot R.
            %
            % Notes::
            % - The link L can belong to any of the Link subclasses.
            % - Extends to arbitrary number of objects, eg. R+L1+L2+L3+L4.
            %
            % See also Link.plus.
            assert( isa(L, 'Link'), 'RTB:SerialLink: second operand for + operator must be a Link class');
            
            R.links = [R.links L];
            r2 = R;
            r2.n = length(R.links);

        end
        
        function r2 = mtimes(r, l)
        %SerialLink.mtimes Concatenate robots
        %
        % R = R1 * R2 is a robot object that is equivalent to mechanically attaching
        % robot R2 to the end of robot R1.
        %
        % Notes::
        % - If R1 has a tool transform or R2 has a base transform these are 
        %   discarded since DH convention does not allow for general intermediate
        %   transformations.
            if isa(l, 'SerialLink')
                r2 = SerialLink([r l]);
            elseif isa(l, 'Link')
                r2 = SerialLink(r);
                r2.links = [r2.links l];
                r2.n = length(r2.links);
            end
        end

        function display(r)
        %SerialLink.display Display parameters
        %
        % R.display() displays the robot parameters in human-readable form.
        %
        % Notes::
        % - This method is invoked implicitly at the command line when the result 
        %   of an expression is a SerialLink object and the command has no trailing
        %   semicolon.
        %
        % See also SerialLink.char, SerialLink.dyn.

            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            if loose
                disp(' ');
            end
            disp(char(r))
            if loose
                disp(' ');
            end
        end

        function s = char(robot)
        %SerialLink.char Convert to string
        %
        % S = R.char() is a string representation of the robot's kinematic parameters,
        % showing DH parameters, joint structure, comments, gravity vector, base and 
        % tool transform.

            s = '';
            for j=1:length(robot)
                r = robot(j);

                % informational line
                info = '';
                if r.mdh
                    info = strcat(info, 'modDH');
                else
                    info = strcat(info, 'stdDH');
                end
                if r.fast
                    info = strcat(info, ', fastRNE');
                else
                    info = strcat(info, ', slowRNE');
                end
                if r.issym
                    info = strcat(info, ', Symbolic');;
                end
                manuf = '';
                if ~isempty(r.manufacturer)
                    manuf = [' [' r.manufacturer ']'];
                end

                s = sprintf('%s%s:: %d axis, %s, %s', r.name, manuf, r.n, r.config, info);

                % comment and other info
                line = '';
%                 if ~isempty(r.manufacturer)
%                     line = strcat(line, sprintf(' from %s;', r.manufacturer));
%                 end
                if ~isempty(r.comment)
                    line = strcat(line, sprintf(' - %s;', r.comment));
                end
                if ~isempty(line)
                s = char(s, line);
                end

                % link parameters
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');
                s = char(s, '| j |     theta |         d |         a |     alpha |    offset |');
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');
                s = char(s, char(r.links, true));
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');


                % gravity, base, tool
%                s_grav = horzcat(char('grav = ', ' ', ' '), mat2str(r.gravity'));
%                 s_grav = char(s_grav, ' ');
%                 s_base = horzcat(char('  base = ',' ',' ', ' '), mat2str(r.base));
% 
%                 s_tool = horzcat(char('   tool =  ',' ',' ', ' '), mat2str(r.tool));
% 
%                 line = horzcat(s_grav, s_base, s_tool);
                %s = char(s, sprintf('gravity: (%g, %g, %g)', r.gravity));
                if ~isidentity(r.base)
                    s = char(s, ['base:    ' trprint(r.base.T, 'xyz')]);
                end
                if ~isidentity(r.tool)
                    s = char(s, ['tool:    ' trprint(r.tool.T, 'xyz')]);
                end

                if j ~= length(robot)
                    s = char(s, ' ');
                end
            end
        end
        
        function T = A(r, joints, q)
        %SerialLink.A Link transformation matrices
        %
        % S = R.A(J, Q) is an SE3 object (4x4) that transforms between link frames
        % for the J'th joint.  Q is a vector (1xN) of joint variables. For:
        %  - standard DH parameters, this is from frame {J-1} to frame {J}.
        %  - modified DH parameters, this is from frame {J} to frame {J+1}.
        %
        % S = R.A(JLIST, Q) as above but is a composition of link transform
        % matrices given in the list JLIST, and the joint variables are taken from
        % the corresponding elements of Q.
        %
        % Exmaples::
        % For example, the link transform for joint 4 is
        %          robot.A(4, q4)
        %
        % The link transform for  joints 3 through 6 is
        %          robot.A(3:6, q)
        % where q is 1x6 and the elements q(3) .. q(6) are used.
        %
        % Notes::
        % - Base and tool transforms are not applied.
        %
        % See also Link.A.
            T = SE3;
            
            for joint=joints
                T = T * r.links(joint).A(q(joint));
            end
        end
        
        function q = getpos(robot)
            %SerialLink.getpos Get joint coordinates from graphical display
            %
            % q = R.getpos() returns the joint coordinates set by the last plot or
            % teach operation on the graphical robot.
            %
            % See also SerialLink.plot, SerialLink.teach.
            
            rhandles = findobj('Tag', robot.name);
            
            % find the graphical element of this name
            if isempty(rhandles)
                error('RTB:getpos:badarg', 'No graphical robot of this name found');
            end
            % get the info from its Userdata
            info = get(rhandles(1), 'UserData');
            
            % the handle contains current joint angles (set by plot)
            if ~isempty(info.q)
                q = info.q;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   set/get methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        function set.tool(r, v)
            if isempty(v)
                r.base = SE(3);
            else
                r.tool = SE3(v);
            end
        end
        
        function set.fast(r,v)
            if v && exist('frne') == 3
                r.fast = true;
            else
                r.fast = false;
            end
        end

        function set.base(r, v)
            if isempty(v)
                r.base = SE3();
            else
                r.base = SE3(v);
            end
        end

        function v = get.d(r)
            v = [r.links.d];
        end
        function v = get.a(r)
            v = [r.links.a];
        end
        function v = get.theta(r)
            v = [r.links.theta];
        end
        function v = get.alpha(r)
            v = [r.links.alpha];
        end
        
        function set.offset(r, v)
            if length(v) ~= length(v)
                error('offset vector length must equal number DOF');
            end
            L = r.links;
            for i=1:r.n
                L(i).offset = v(i);
            end
            r.links = L;
        end

        function v = get.offset(r)
            v = [r.links.offset];
        end

        function set.qlim(r, v)
            if numrows(v) ~= r.n
                error('insufficient rows in joint limit matrix');
            end
            L = r.links;
            for i=1:r.n
                L(i).qlim = v(i,:);
            end
            r.links = L;
        end

        function v = get.qlim(r)
            L = r.links;
            v = zeros(r.n, 2);
            for i=1:r.n
                if isempty(L(i).qlim)
                    if L(i).isrevolute
                        v(i,:) = [-pi pi];
                    else
                        v(i,:) = [-Inf Inf];
                    end
                else
                    v(i,:) = L(i).qlim;
                end
            end
        end

        function set.gravity(r, v)
            if isvec(v, 3)
                r.gravity = v(:);
            else
                error('gravity must be a 3-vector');
            end
        end

        function v = get.config(r)
        %SerialLink.config Returnt the joint configuration string
            v = '';
            for i=1:r.n
                v(i) = r.links(i).type;
            end
        end

        % general methods

        function v = islimit(r,q)
        %SerialLink.islimit Joint limit test
        %
        % V = R.islimit(Q) is a vector of boolean values, one per joint, 
        % false (0) if Q(i) is within the joint limits, else true (1).
        %
        % Notes::
        % - Joint limits are not used by many methods, exceptions being:
        %   - ikcon() to specify joint constraints for inverse kinematics.
        %   - by plot() for prismatic joints to help infer the size of the
        %     workspace
        %
        % See also Link.islimit.
            L = r.links;
            if length(q) ~= r.n
                error('argument for islimit method is wrong length');
            end
            v = zeros(r.n, 2);
            for i=1:r.n
                v(i,:) = L(i).islimit(q(i));
            end
        end

        function v = isspherical(r)
        %SerialLink.isspherical Test for spherical wrist
        %
        % R.isspherical() is true if the robot has a spherical wrist, that is, the
        % last 3 axes are revolute and their axes intersect at a point.
        %
        % See also SerialLink.ikine6s.
            L = r.links(end-2:end);
            
            alpha = [-pi/2 pi/2];
            v =  all([L(1:2).a] == 0) && ...
                (L(2).d == 0) && ...
                (all([L(1:2).alpha] == alpha) || all([L(1:2).alpha] == -alpha)) && ...
                all([L(1:3).isrevolute]);
        end
        
        function v = isconfig(r, s)
            %SerialLink.isconfig Test for particular joint configuration
            %
            % R.isconfig(s) is true if the robot has the joint configuration string
            % given by the string s.
            %
            % Example:
            %        robot.isconfig('RRRRRR');
            %
            % See also SerialLink.config.
            v = strcmpi(r.config, s);
        end
        
        function payload(r, m, p)
            %SerialLink.payload Add payload mass
            %
            % R.payload(M, P) adds a payload with point mass M at position P 
            % in the end-effector coordinate frame.
            %
            % R.payload(0) removes added payload
            %
            % Notes::
            % - An added payload will affect the inertia, Coriolis and gravity terms.
            % - Sets, rather than adds, the payload.  Mass and CoM of the last link is
            %   overwritten.
            %
            % See also SerialLink.rne, SerialLink.gravload.

            lastlink = r.links(r.n);

            if nargin == 3
                lastlink.m = m;
                lastlink.r = p;
            elseif nargin == 2 && m == 0
                % clear/reset the payload
                lastlink.m = m;
            end
        end
        
        function t = jointdynamics(robot, q, qd)
            %SerialLink.jointdyamics Transfer function of joint actuator
            %
            % TF = R.jointdynamic(Q) is a vector of N continuous-time transfer function
            % objects that represent the transfer function 1/(Js+B) for each joint
            % based on the dynamic parameters of the robot and the configuration Q
            % (1xN).  N is the number of robot joints.
            %
            % % TF = R.jointdynamic(Q, QD) as above but include the linearized effects
            % of Coulomb friction when operating at joint velocity QD (1xN).
            %
            % Notes::
            % - Coulomb friction is ignoredf.
            %
            % See also tf, SerialLink.rne.
            
            for j=1:robot.n
                link = robot.links(j);
                
                % compute inertia for this joint
                zero = zeros(1, robot.n);
                qdd = zero; qdd(j) = 1;
                M = robot.rne(q, zero, qdd, 'gravity', [0 0 0]);
                J = link.Jm + M(j)/abs(link.G)^2;
                
                % compute friction
                B = link.B;
                if nargin == 3
                    % add linearized Coulomb friction at the operating point
                    if qd > 0
                        B = B + link.Tc(1)/qd(j);
                    elseif qd < 0
                        B = B + link.Tc(2)/qd(j);
                    end
                end
                t(j) = tf(1, [J B]);

            end
        end
        
        function jt = jtraj(r, T1, T2, t, varargin)
            %SerialLink.jtraj Joint space trajectory
            %
            % Q = R.jtraj(T1, T2, K, OPTIONS) is a joint space trajectory (KxN) where
            % the joint coordinates reflect motion from end-effector pose T1 to T2 in K
            % steps, where N is the number of robot joints. T1 and T2 are SE3 objects or homogeneous transformation
            % matrices (4x4). The trajectory Q has one row per time step, and one
            % column per joint.
            %
            % Options::
            % 'ikine',F   A handle to an inverse kinematic method, for example
            %             F = @p560.ikunc.  Default is ikine6s() for a 6-axis spherical
            %             wrist, else ikine().
            %
            % Notes::
            % - Zero boundary conditions for velocity and acceleration are assumed.
            % - Additional options are passed as trailing arguments to the
            %   inverse kinematic function, eg. configuration options like 'ru'.
            %
            % See also jtraj, SerialLink.ikine, SerialLink.ikine6s.
            if r.isspherical && (r.n == 6)
                opt.ikine = @r.ikine6s;
            else
                opt.ikine = @r.ikine;
            end
            [opt,args] = tb_optparse(opt, varargin);
            
            q1 = opt.ikine(T1, args{:});
            q2 = opt.ikine(T2, args{:});
            
            jt = jtraj(q1, q2, t);
        end
        
        
        function dyn(r, j)
            %SerialLink.dyn Print inertial properties
            %
            % R.dyn() displays the inertial properties of the SerialLink object in a multi-line
            % format.  The properties shown are mass, centre of mass, inertia, gear ratio,
            % motor inertia and motor friction.
            %
            % R.dyn(J) as above but display parameters for joint J only.
            %
            % See also Link.dyn.
            if nargin == 2
                r.links(j).dyn()
            else
                r.links.dyn();
            end
        end
        
        function sr = sym(r)
            sr = SerialLink(r);
            for i=1:r.n
                sr.links(i) = r.links(i).sym;
            end
        end

                 
        function p = isprismatic(robot)
        %SerialLink.isprismatic identify prismatic joints
        %
        % X = R.isprismatic is a list of logical variables, one per joint, true if
        % the corresponding joint is prismatic, otherwise false.
        %
        % See also Link.isprismatic, SerialLink.isrevolute.
            p = robot.links.isprismatic();
        end
        
        function p = isrevolute(robot)
        %SerialLink.isrevolute identify revolute joints
        %
        % X = R.isrevolute is a list of logical variables, one per joint, true if
        % the corresponding joint is revolute, otherwise false.
        %
        % See also Link.isrevolute, SerialLink.isprismatic.
            p = robot.links.isrevolute();
        end
        
        function qdeg = todegrees(robot, q)
        %SerialLink.todegrees Convert joint angles to degrees
        %
        % Q2 = R.todegrees(Q) is a vector of joint coordinates where those elements
        % corresponding to revolute joints are converted from radians to degrees.
        % Elements corresponding to prismatic joints are copied unchanged.
        %
        % See also SerialiLink.toradians.
            k = robot.isrevolute;
            qdeg = q;
            qdeg(:,k) = qdeg(:,k) * 180/pi;
        end
        
        function qrad = toradians(robot, q)
        %SerialLink.toradians Convert joint angles to radians
        %
        % Q2 = R.toradians(Q) is a vector of joint coordinates where those elements
        % corresponding to revolute joints are converted from degrees to radians.
        % Elements corresponding to prismatic joints are copied unchanged.
        %
        % See also SerialiLink.todegrees.
            k = robot.isrevolute;
            qrad = q;
            qrad(:,k) = qrad(:,k) * pi/180;
        end
        
        function J = jacobn(robot, varargin)
            warning('RTB:SerialLink:deprecated', 'Use jacobe instead of jacobn');
            J = robot.jacobe(varargin{:});
        end
        
        function rmdh = MDH(r)
        %SerialLink.MDH  Convert standard DH model to modified
        %
        % rmdh = R.MDH() is a SerialLink object that represents the same kinematics
        % as R but expressed using modified DH parameters.
        %
        % Notes::
        % - can only be applied to a model expressed with standard DH parameters.
        %
        % See also:  DH
        
            assert(isdh(r), 'RTB:SerialLink:badmodel', 'this method can only be applied to a model with standard DH parameters');
            
            % first joint
            switch r.config(1)
                case 'R'
                    link(1) = Link('modified', 'revolute', ...
                        'd', r.links(1).d, ...
                        'offset', r.links(1).offset, ...
                        'qlim', r.links(1).qlim );
                case 'P'
                    link(1) = Link('modified', 'prismatic', ...
                        'theta', r.links(1).theta, ...
                        'offset', r.links(1).offset, ...
                        'qlim', r.links(1).qlim );
            end

            % middle joints
            for i=2:r.n
                switch r.config(i)
                    case 'R'
                        link(i) = Link('modified', 'revolute', ...
                            'a', r.links(i-1).a, ...
                            'alpha', r.links(i-1).alpha, ...
                            'd', r.links(i).d, ...
                            'offset', r.links(i).offset, ...
                            'qlim', r.links(i).qlim );
                    case 'P'
                        link(i) = Link('modified', 'prismatic', ...
                            'a', r.links(i-1).a, ...
                            'alpha', r.links(i-1).alpha, ...
                            'theta', r.links(i).theta, ...
                            'offset', r.links(i).offset, ...
                            'qlim', r.links(i).qlim );
                end
            end
            
            % last joint
            tool = SE3(r.links(r.n).a, 0, 0) * SE3.Rx(r.links(r.n).alpha) * r.tool;
            
            rmdh = SerialLink(link, 'base', r.base, 'tool', tool);
        end
        
        function rdh = DH(r)
        %SerialLink.DH  Convert modified DH model to standard
        %
        % rmdh = R.DH() is a SerialLink object that represents the same kinematics
        % as R but expressed using standard DH parameters.
        %
        % Notes::
        % - can only be applied to a model expressed with modified DH parameters.
        %
        % See also:  MDH
            
            assert(ismdh(r), 'RTB:SerialLink:badmodel', 'this method can only be applied to a model with modified DH parameters');
            
            base = r.base * SE3(r.links(1).a, 0, 0) * SE3.Rx(r.links(1).alpha);

            % middle joints
            for i=1:r.n-1
                switch r.config(i)
                    case 'R'
                        link(i) = Link('standard', 'revolute', ...
                            'a', r.links(i+1).a, ...
                            'alpha', r.links(i+1).alpha, ...
                            'd', r.links(i).d, ...
                            'offset', r.links(i).offset, ...
                            'qlim', r.links(i).qlim );
                    case 'P'
                        link(i) = Link('standard', 'prismatic', ...
                            'a', r.links(i+1).a, ...
                            'alpha', r.links(i+1).alpha, ...
                            'theta', r.links(i).theta, ...
                            'offset', r.links(i).offset, ...
                            'qlim', r.links(i).qlim );
                end
            end
            
            % last joint
            switch r.config(r.n)
                case 'R'
                    link(r.n) = Link('standard', 'revolute', ...
                        'd', r.links(r.n).d, ...
                        'offset', r.links(r.n).offset, ...
                        'qlim', r.links(r.n).qlim );
                case 'P'
                    link(r.n) = Link('standard', 'prismatic', ...
                        'theta', r.links(r.n).theta, ...
                        'offset', r.links(r.n).offset, ...
                        'qlim', r.links(r.n).qlim );
            end
            
            rdh = SerialLink(link, 'base', base, 'tool', r.tool);
        end
        
        function [tw,T0] = twists(r, q)
        %SerialLink.twists Joint axis twists
        %
        % [TW,T0] = R.twists(Q) is a vector of Twist objects (1xN) that represent
        % the axes of the joints for the robot with joint coordinates Q (1xN).  T0
        % is an SE3 object representing the pose of the tool.
        %
        % [TW,T0] = R.twists() as above but the joint coordinates are taken to be
        % zero.
        %
        % Notes::
        % - [TW,T0] is the product of exponential representation of the robot's
        %   forward kinematics:  prod( [TW.exp(Q) T0] )
        %
        % See also Twist.
            if nargin < 2
                q = zeros(1, r.n);
            end
            
            [Tn,T] = r.fkine( q );
            if r.isdh
                % DH case
                for i=1:r.n
                    if i == 1
                        tw(i) = Twist( r.links(i).type, [0 0 1], [0 0 0]);
                    else
                        tw(i) = Twist( r.links(i).type, T(i-1).a, T(i-1).t);
                    end
                end
            else
                % MDH case
                for i=1:r.n
                    tw(i) = Twist( r.links(i).type, T(i).a, T(i).t);
                end
            end
            
            if nargout > 1
                T0 = Tn;
            end
        end
        
        function v = ismdh(r)
        %SerialLink.ismdh Test if SerialLink object has a modified DH model
        %
        % v = R.ismdh() is true if the SerialLink manipulator R has a modified DH model
        %
        % See also: isdh
            v = logical(r.mdh);
        end
        
        function v = isdh(r)
        %SerialLink.isdh Test if SerialLink object has a standard DH model
        %
        % v = R.isdh() is true if the SerialLink manipulator R has a standard DH model
        %
        % See also: ismdh
            v = ~r.mdh;
        end
        
    end % methods

end % classdef

% utility function
function s = mat2str(m)
    if isa(m, 'sym')
        % turn a symbolic matrix into a string (it shouldnt be so hard)
        ss = cell(size(m)); colwidth = zeros(1, numcols(m));
        for j=1:numcols(m)
            for i=1:numrows(m)
                ss{i,j} = char(m(i,j));
                colwidth(j) = max( colwidth(j), length(ss{i,j}));
            end
        end
        s = '';
        for i=1:numrows(m)
            line = '';
            for j=1:numcols(m)
                % append the element with a blank
                line = [line ' ' ss{i,j}];
                % pad it out to column width
                for k=0:colwidth(j)-length(ss{i,j})
                    line = [line ' '];
                end
            end
            s = strvcat(s, line);
        end
    else
        m(abs(m)<eps) = 0;
        s = num2str(m);
    end
end

