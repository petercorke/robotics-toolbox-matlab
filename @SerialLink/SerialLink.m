%SerialLink Serial-link robot class
%
% A concrete class that represents a serial-link arm-type robot.  The
% mechanism is described using Denavit-Hartenberg parameters, one set
% per joint.
%
% Methods::
%
%  plot          display graphical representation of robot
%  plot3d        display 3D graphical model of robot
%  teach         drive the graphical robot
%  getpos        get position of graphical robot
%-
%  jtraj         a joint space trajectory
%-
%  edit          display and edit kinematic and dynamic parameters
%-
%  isspherical   test if robot has spherical wrist
%  islimit       test if robot at joint limit
%  isconfig      test robot joint configuration
%-
%  fkine         forward kinematics
%  A             link transforms
%  trchain       forward kinematics as a chain of elementary transforms
%-
%  ikine6s       inverse kinematics for 6-axis spherical wrist revolute robot
%  ikine         inverse kinematics using iterative numerical method
%  ikunc         inverse kinematics using optimisation
%  ikcon         inverse kinematics using optimisation with joint limits
%  ikine_sym     analytic inverse kinematics obtained symbolically
%-
%  jacob0        Jacobian matrix in world frame
%  jacobn        Jacobian matrix in tool frame
%  jacob_dot     Jacobian derivative
%  maniplty      manipulability
%  vellipse      display velocity ellipsoid
%  fellipse      display force ellipsoid
%  qmincon       null space motion to centre joints between limits
%-
%  accel         joint acceleration
%  coriolis      Coriolis joint force
%  dyn           show dynamic properties of links
%  friction      friction force
%  gravload      gravity joint force
%  inertia       joint inertia matrix
%  cinertia      Cartesian inertia matrix
%  nofriction    set friction parameters to zero
%  rne           inverse dynamics
%  fdyn          forward dynamics
%-
%  payload       add a payload in end-effector frame
%  perturb       randomly perturb link dynamic parameters
%  gravjac       gravity load and Jacobian
%  paycap        payload capacity
%  pay           payload effect
%-
%  sym           a symbolic version of the object
%  gencoords     symbolic generalized coordinates
%  genforces     symbolic generalized forces
%  issym         test if object is symbolic
%
% Properties (read/write)::
%
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
%
%  n           number of joints
%  config      joint configuration string, eg. 'RRRRRR'
%  mdh         kinematic convention boolean (0=DH, 1=MDH)
%  theta       kinematic: joint angles (1xN)
%  d           kinematic: link offsets (1xN)
%  a           kinematic: link lengths (1xN)
%  alpha       kinematic: link twists (1xN)
%
% Overloaded operators::
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

% TODO:
% fix payload as per GG discussion

classdef SerialLink < handle

    properties
        name
        gravity
        base
        tool
        manuf
        comment

        plotopt  % options for the plot method, follow plot syntax
        
        fast    % mex version of rne detected
        
        interface   % interface to a real robot platform
        
        ikineType
        
        trail   % to support trail option
        
        % to support plot method
        moviepath
        framenum
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
        n
        links
        mdh
        T
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
        function r = SerialLink(L, varargin)
        %SerialLink Create a SerialLink robot object
        %
        % R = SerialLink(LINKS, OPTIONS) is a robot object defined by a vector 
        % of Link class objects which can be instances of Link, Revolute,
        % Prismatic, RevoluteMDH or PrismaticMDH.
        %
        % R = SerialLink(OPTIONS) is a null robot object with no links.
        %
        % R = SerialLink([R1 R2 ...], OPTIONS) concatenate robots, the base of
        % R2 is attached to the tip of R1.  Can also be written R1*R2 etc.
        %
        % R = SerialLink(R1, options) is a deep copy of the robot object R1, 
        % with all the same properties.
        %
        % R = SerialLink(DH, OPTIONS) is a robot object with kinematics defined
        % by the matrix DH which has one row per joint and each row is
        % [theta d a alpha] and joints are assumed revolute.  An optional 
        % fifth column sigma indicate revolute (sigma=0, default) or 
        % prismatic (sigma=1).
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

            r.name = 'noname';
            r.manuf = '';
            r.comment = '';
            r.links = [];
            r.n = 0;
            r.mdh = 0;
            r.gravity = [0; 0; 9.81];
            r.base = eye(4,4);
            r.tool = eye(4,4);
            
            r.plotopt = {};
            r.plotopt3d = {};

            if exist('frne') == 3
                r.fast = true;
            else
                r.fast = false;
            end
            
            % process the rest of the arguments in key, value pairs
            opt.name = [];
            opt.comment = [];
            opt.manufacturer = [];
            opt.base = [];
            opt.tool = [];
            opt.offset = [];
            opt.qlim = [];
            opt.plotopt = [];
            opt.ikine = [];
            opt.fast = r.fast;
            opt.modified = false;

            [opt,out] = tb_optparse(opt, varargin);
            if ~isempty(out)
                error('unknown option <%s>', out{1});
            end

            if nargin == 0
                % zero argument constructor, sets default values
                return;
            else
                % at least one argument, either a robot or link array

                if isa(L, 'SerialLink')
                    if length(L) == 1
                        % clone the passed robot
                        for j=1:L.n
                            newlinks(j) = copy(L.links(j));
                        end
                        r.links = newlinks;
                        r.name = [L.name ' copy'];
                        r.base = L.base;
                        r.tool = L.tool;
                    else
                        % compound the robots in the vector
                        r = L(1);
                        for k=2:length(L)
                            r = r * L(k);
                        end
                        r.base = L(1).base;
                        r.tool = L(end).tool;
                    end
                elseif isa(L, 'Link')
                    r.links = L;    % attach the links
                elseif numcols(L) >= 4 && (isa(L, 'double') || isa(L, 'sym'))
                    % legacy matrix
                    dh_dyn = L;
                    clear L
                    for j=1:numrows(dh_dyn)
                        if opt.modified 
                            L(j) =        Link(dh_dyn(j,:), 'modified');
                        else
                            L(j) =        Link(dh_dyn(j,:));
                            
                        end
%                         L(j).theta =  dh_dyn(j,1);
%                         L(j).d =      dh_dyn(j,2);
%                         L(j).a =      dh_dyn(j,3);
%                         L(j).alpha =  dh_dyn(j,4);

%                         if numcols(dh_dyn) > 4
%                             L(j).sigma = dh_dyn(j,5);
%                         end
                    end
                    r.links = L;
                else
                    error('unknown type passed to robot');
                end
                r.n = length(r.links);
            end



            % copy the properties to robot object
            p = properties(r);
            for i=1:length(p)
                if isfield(opt, p{i}) && ~isempty(getfield(opt, p{i}))
                    r.(p{i}) = getfield(opt, p{i});
                end
            end
            r.ikineType = opt.ikine;


            % set the robot object mdh status flag
            mdh = [r.links.mdh];
            if all(mdh == 0)
                r.mdh = mdh(1);
            elseif all (mdh == 1)
                r.mdh = mdh(1);
            else
                error('robot has mixed D&H links conventions');
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

        function r2 = mtimes(r, l)
        %SerialLink.mtimes Concatenate robots
        %
        % R = R1 * R2 is a robot object that is equivalent to mechanically attaching
        % robot R2 to the end of robot R1.
        %
        % Notes::
        % - If R1 has a tool transform or R2 has a base transform these are 
        %   discarded since DH convention does not allow for arbitrary intermediate
        %   transformations.
            if isa(l, 'SerialLink')
                r2 = SerialLink(r);
                r2.links = [r2.links l.links];
                r2.base = r.base;
                r2.n = length(r2.links);
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
                if r.mdh
                    convention = 'modDH';
                else
                    convention = 'stdDH';
                end
                if r.fast
                    fast = 'fastRNE';
                else
                    fast = 'slowRNE';
                end
                s = sprintf('%s (%d axis, %s, %s, %s)', r.name, r.n, r.config, convention, fast);

                % comment and other info
                line = '';
                if ~isempty(r.manuf)
                    line = strcat(line, sprintf(' %s;', r.manuf));
                end
                if ~isempty(r.comment)
                    line = strcat(line, sprintf(' %s;', r.comment));
                end
                s = char(s, line);

                % link parameters
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');
                s = char(s, '| j |     theta |         d |         a |     alpha |    offset |');
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');
                s = char(s, char(r.links, true));
                s = char(s, '+---+-----------+-----------+-----------+-----------+-----------+');


                % gravity, base, tool
                s_grav = horzcat(char('grav = ', ' ', ' '), mat2str(r.gravity));
                s_grav = char(s_grav, ' ');
                s_base = horzcat(char('  base = ',' ',' ', ' '), mat2str(r.base));

                s_tool = horzcat(char('   tool =  ',' ',' ', ' '), mat2str(r.tool));

                line = horzcat(s_grav, s_base, s_tool);

                s = char(s, ' ', line);
                if j ~= length(robot)
                    s = char(s, ' ');
                end
            end
        end
        
        function T = A(r, joints, q)
        %SerialLink.A Link transformation matrices
        %
        % S = R.A(J, QJ) is an SE(3) homogeneous transform (4x4) that transforms
        % from link frame {J-1} to frame {J} which is a function of the J'th joint
        % variable QJ.
        %
        % S = R.A(jlist, q) as above but is a composition of link transform
        % matrices given in the list JLIST, and the joint variables are taken from
        % the corresponding elements of Q.
        %
        % Exmaples::
        % For example, the link transform for joint 4 is
        %          robot.A(4, q4)
        %
        % The link transform for  joints 3 through 6 is
        %          robot.A([3 4 5 6], q)
        % where q is 1x6 and the elements q(3) .. q(6) are used.
        %
        % Notes::
        % - base and tool transforms are not applied.
            T = eye(4,4);
            
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
                r.base = eye(4,4);
            elseif ~ishomog(v)
                error('tool must be a homogeneous transform');
            else
                r.tool = v;
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
                r.base = eye(4,4);
            elseif ~ishomog(v)
                error('base must be a homogeneous transform');
            else
                r.base = v;
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
            v = '';
            for i=1:r.n
                v(i) = r.links(i).RP;
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
        % - Joint limits are purely advisory and are not used in any
        %   other function.  Just seemed like a useful thing to include...
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
                all([L(1:3).sigma] == 0);
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
            v = strcmpi(r.config(), s);
        end
        
        function payload(r, m, p)
        %SerialLink.payload Add payload mass
        %
        % R.payload(M, P) adds a payload with point mass M at position P 
        % in the end-effector coordinate frame.
        %
        % Notes::
        % - An added payload will affect the inertia, Coriolis and gravity terms.
        %
        % See also SerialLink.rne, SerialLink.gravload.
            lastlink = r.links(r.n);
            lastlink.m = m;
            lastlink.r = p;
        end
        
        function jt = jtraj(r, T1, T2, t, varargin)
            %SerialLink.jtraj Joint space trajectory
            %
            % Q = R.jtraj(T1, T2, K, OPTIONS) is a joint space trajectory (KxN) where
            % the joint coordinates reflect motion from end-effector pose T1 to T2 in K
            % steps  with default zero boundary conditions for velocity and
            % acceleration. The trajectory Q has one row per time step, and one column
            % per joint, where N is the number of robot joints.
            %
            % Options::
            % 'ikine',F   A handle to an inverse kinematic method, for example
            %             F = @p560.ikunc.  Default is ikine6s() for a 6-axis spherical
            %             wrist, else ikine().
            %
            % Additional options are passed as trailing arguments to the inverse
            % kinematic function.
            %
            % See also jtraj, SerialLink.ikine, SerialLink.ikine6s.
            if r.isspherical && (r.n == 6)
                opt.ikine = @r.ikine;
            else
                opt.ikine = @r.ikine6s;
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
