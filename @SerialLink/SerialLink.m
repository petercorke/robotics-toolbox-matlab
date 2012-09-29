%SerialLink Serial-link robot class
%
% A concrete class that represents a serial-link arm-type robot.  The
% mechanism is described using Denavit-Hartenberg parameters, one set
% per joint.
%
% Methods::
%
%  plot          display graphical representation of robot
%  teach         drive the graphical robot
%  isspherical   test if robot has spherical wrist
%  islimit       test if robot at joint limit
%
%  fkine         forward kinematics
%  ikine6s       inverse kinematics for 6-axis spherical wrist revolute robot
%  ikine3        inverse kinematics for 3-axis revolute robot
%  ikine         inverse kinematics using iterative method
%  jacob0        Jacobian matrix in world frame
%  jacobn        Jacobian matrix in tool frame
%  maniplty      manipulability
%
%  jtraj         a joint space trajectory
%
%  accel         joint acceleration
%  coriolis      Coriolis joint force
%  dyn           show dynamic properties of links
%  fdyn          joint motion
%  friction      friction force
%  gravload      gravity joint force
%  inertia       joint inertia matrix
%  nofriction    set friction parameters to zero
%  rne           joint torque/force
%  payload       add a payload in end-effector frame
%  perturb       randomly perturb link dynamic parameters
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
%
% Object properties (read only)::
%
%  n           number of joints
%  config      joint configuration string, eg. 'RRRRRR'
%  mdh         kinematic convention boolean (0=DH, 1=MDH)
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
%
% http://www.petercorke.com

classdef SerialLink < handle

    properties
        name
        gravity
        base
        tool
        manuf
        comment

        plotopt
        lineopt
        shadowopt
        
        qteach

        fast_rne    % mex version of rne detected

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
    end

    methods
        function r = SerialLink(L, varargin)
        %SerialLink Create a SerialLink robot object
        %
        % R = SerialLink(LINKS, OPTIONS) is a robot object defined by a vector 
        % of Link objects.
        %
        % R = SerialLink(DH, OPTIONS) is a robot object with kinematics defined
        % by the matrix DH which has one row per joint and each row is
        % [theta d a alpha] and joints are assumed revolute.  An optional 
        % fifth column sigma indicate revolute (sigma=0, default) or 
        % prismatic (sigma=1).
        %
        % R = SerialLink(OPTIONS) is a null robot object with no links.
        %
        % R = SerialLink([R1 R2 ...], OPTIONS) concatenate robots, the base of
        % R2 is attached to the tip of R1.
        %
        % R = SerialLink(R1, options) is a deep copy of the robot object R1, 
        % with all the same properties.
        %
        % Options::
        %
        %  'name', name            set robot name property
        %  'comment', comment      set robot comment property
        %  'manufacturer', manuf   set robot manufacturer property
        %  'base', base            set base transformation matrix property
        %  'tool', tool            set tool transformation matrix property
        %  'gravity', g            set gravity vector property
        %  'plotopt', po           set plotting options property
        %
        % Examples::
        %
        % Create a 2-link robot
        %      L(1) = Link([ 0     0   a1  0], 'standard');
        %      L(2) = Link([ 0     0   a2  0], 'standard');
        %      twolink = SerialLink(L, 'name', 'two link');
        %
        % Robot objects can be concatenated in two ways
        %      R = R1 * R2;
        %      R = SerialLink([R1 R2]);
        %
        % Note::
        % - SerialLink is a reference object, a subclass of Handle object.
        % - SerialLink objects can be used in vectors and arrays
        % - When robots are concatenated (either syntax) the intermediate base and
        %   tool transforms are removed since general constant transforms cannot 
        %   be represented in Denavit-Hartenberg notation.
        %
        % See also Link, SerialLink.plot.

            r.name = 'noname';
            r.manuf = '';
            r.comment = '';
            r.links = [];
            r.n = 0;
            r.mdh = 0;
            r.gravity = [0; 0; 9.81];
            r.base = eye(4,4);
            r.tool = eye(4,4);

            r.lineopt = {'Color', 'black', 'Linewidth', 4};
            r.shadowopt = {'Color', 0.7*[1 1 1], 'Linewidth', 3};
            r.plotopt = {};

            if exist('frne') == 3
                r.fast_rne = true;
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
                    else
                        % compound the robots in the vector
                        r = L(1);
                        for k=2:length(L)
                            r = r * L(k);
                        end
                    end
                elseif isa(L, 'Link')
                    r.links = L;    % attach the links
                elseif isa(L, 'double')
                    % legacy matrix
                    dh_dyn = L;
                    clear L
                    for j=1:numrows(dh_dyn)
                        L(j) =        Link();
                        L(j).theta =  dh_dyn(j,1);
                        L(j).d =      dh_dyn(j,2);
                        L(j).a =      dh_dyn(j,3);
                        L(j).alpha =  dh_dyn(j,4);

                        if numcols(dh_dyn) > 4
                            L(j).sigma = dh_dyn(j,5);
                        end
                    end
                    r.links = L;
                else
                    error('unknown type passed to robot');
                end
                r.n = length(r.links);
            end

            % process the rest of the arguments in key, value pairs
            opt.name = 'robot';
            opt.comment = [];
            opt.manufacturer = [];
            opt.base = [];
            opt.tool = [];
            opt.offset = [];
            opt.qlim = [];
            opt.plotopt = [];

            [opt,out] = tb_optparse(opt, varargin);
            if ~isempty(out)
                error('unknown option <%s>', out{1});
            end

            % copy the properties to robot object
            p = properties(r);
            for i=1:length(p)
                if isfield(opt, p{i}) && ~isempty(getfield(opt, p{i}))
                    r.(p{i}) = getfield(opt, p{i});
                end
            end

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
                s = sprintf('%s (%d axis, %s, %s)', r.name, r.n, r.config, convention);

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
                s = char(s, '+---+-----------+-----------+-----------+-----------+');
                s = char(s, '| j |     theta |         d |         a |     alpha |');
                s = char(s, '+---+-----------+-----------+-----------+-----------+');
                s = char(s, char(r.links, true));
                s = char(s, '+---+-----------+-----------+-----------+-----------+');

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

        function set.base(r, v)
            if isempty(v)
                r.base = eye(4,4);
            elseif ~ishomog(v)
                error('base must be a homogeneous transform');
            else
                r.base = v;
            end
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
                r.gravity = v;
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

            v = false;
            % first test that all lengths are zero
            if ~all( [L(1).a L(2).a L(3).a L(2).d L(3).d] == 0 )
                return
            end

            if (abs(L(1).alpha) == pi/2) && (abs(L(1).alpha + L(2).alpha) < eps)
                v = true;
                return;
            end
        end
        
        function payload(r, m, p)
        %SerialLink.payload Add payload mass
        %
        % R.payload(M, P) adds a payload with point mass M at position P 
        % in the end-effector coordinate frame.
        %
        % See also SerialLink.rne, SerialLink.gravload.
            lastlink = r.links(r.n);
            lastlink.m = m;
            lastlink.r = p;
        end
        
        function jt = jtraj(r, T1, T2, t, varargin)
        %SerialLink.jtraj Joint space trajectory
        %
        % Q = R.jtraj(T1, T2, K) is a joint space trajectory (KxN) where the joint
        % coordinates reflect motion from end-effector pose T1 to T2 in K steps  with 
        % default zero boundary conditions for velocity and acceleration.  
        % The trajectory Q has one row per time step, and one column per joint,
        % where N is the number of robot joints.
        %
        % Note::
        % - Requires solution of inverse kinematics. R.ikine6s() is used if
        %   appropriate, else R.ikine().  Additional trailing arguments to R.jtraj()
        %   are passed as trailing arugments to these functions.
        %
        % See also jtraj, SerialLink.ikine, SerialLink.ikine6s.
            if r.isspherical && (r.n == 6)
                q1 = r.ikine6s(T1, varargin{:});
                q2 = r.ikine6s(T2, varargin{:});
            else
                q1 = r.ikine(T1, varargin{:});
                q2 = r.ikine(T2, varargin{:});
            end
            
            jt = jtraj(q1, q2, t);
        end


        function dyn(r, j)
        %SerialLink.dyn Display inertial properties
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

        function record(r, q)
            r.qteach = [r.qteach; q];
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
    m(abs(m)<eps) = 0;
    s = num2str(m);
end
