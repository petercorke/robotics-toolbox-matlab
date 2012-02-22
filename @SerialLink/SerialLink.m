%SerialLink Serial-link robot class
%
% r = SerialLink(links, options) is a serial-link robot object from a 
% vector of Link objects.
%
% r = SerialLink(dh, options) is a serial-link robot object from a table
% (matrix) of Denavit-Hartenberg parameters.  The columns of the matrix are
% theta, d, alpha, a.  An optional fifth column sigma indicate 
% revolute (sigma=0, default) or prismatic (sigma=1).
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
% Methods::
%
%  plot          display graphical representation of robot
%  teach         drive the graphical robot
%  fkine         forward kinematics
%  ikine6s       inverse kinematics for 6-axis spherical wrist revolute robot
%  ikine3        inverse kinematics for 3-axis revolute robot
%  ikine         inverse kinematics using iterative method
%  jacob0        Jacobian matrix in world frame
%  jacobn        Jacobian matrix in tool frame
%  jtraj         a joint space trajectory
%  dyn           show dynamic properties of links
%  isspherical   true if robot has spherical wrist
%  islimit       true if robot has spherical wrist
%  payload       add a payload in end-effector frame
%
%  coriolis      Coriolis joint force
%  gravload      gravity joint force
%  inertia       joint inertia matrix
%  accel         joint acceleration
%  fdyn          joint motion
%  rne           joint force
%  perturb       SerialLink object with perturbed parameters
%  showlink      SerialLink object with perturbed parameters
%  friction      SerialLink object with perturbed parameters
%  maniplty      SerialLink object with perturbed parameters
%
% Properties (read/write)::
%
%  links      vector of Link objects
%  gravity    direction of gravity [gx gy gz]
%  base       pose of robot's base 4x4 homog xform
%  tool       robot's tool transform, T6 to tool tip: 4x4 homog xform
%  qlim       joint limits, [qlower qupper] nx2
%  offset     kinematic joint coordinate offsets nx1
%  name       name of robot, used for graphical display
%  manuf      annotation, manufacturer's name
%  comment    annotation, general comment
%
%  plotopt    options for plot(robot), cell array
%
% Object properties (read only)::
%
%  n           number of joints
%  config      joint configuration string, eg. 'RRRRRR'
%  mdh         kinematic convention boolean (0=DH, 1=MDH)
%  islimit     joint limit boolean vector
%
%  q           joint angles from last plot operation
%  handle      graphics handles in object
%
% Note::
%  - SerialLink is a reference object.
%  - SerialLink objects can be used in vectors and arrays
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

    end

    events
        Moved
    end

    properties (SetAccess = private)
        n
        links
        mdh
        handle
        q
        T
    end

    properties (Dependent = true, SetAccess = private)
        config
        %dyn
        dh
    end

    properties (Dependent = true)
        offset
        qlim
    end

    methods
        function r = SerialLink(L, varargin)
        %SerialLink Create a SerialLink robot object
        %
        % R = SerialLink(options) is a null robot object with no links.
        %
        % R = SerialLink(R1, options) is a deep copy of the robot object R1, 
        % with all the same properties.
        %
        % R = SerialLink(DH, options) is a robot object with kinematics defined
        % by the matrix DH which has one row per joint and each row is
        % [theta d a alpha] and joints are assumed revolute.
        %
        % R = SerialLink(LINKS, options) is a robot object defined by a vector 
        % of Link objects.
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
        % Robot objects can be concatenated by:
        %
        %    R = R1 * R2;
        %    R = SerialLink([R1 R2]);
        %
        % which is equivalent to R2 mounted on the end of R1.  Note that tool transform of R1
        % and the base transform of R2 are lost, constant transforms cannot be represented in
        % Denavit-Hartenberg notation.
        %
        % Note::
        %  - SerialLink is a reference object, a subclass of Handle object.
        %  - SerialLink objects can be used in vectors and arrays
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
            r.handle = [];  % graphics handles
            r.q = [];   % current joint angles

            r.lineopt = {'Color', 'black', 'Linewidth', 4};
            r.shadowopt = {'Color', 0.7*[1 1 1], 'Linewidth', 3};
            r.plotopt = {};

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
                error( sprintf('unknown option <%s>', out{1}));
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

        function r2 = copy(r)
            %SerialLink.copy Clone a robot object
            %
            % R2 = R.copy() is a deepcopy of the object R.
            r2 = SerialLink(r);
        end


        function r2 = mtimes(r, l)
        %SerialLink.mtimes Join robots
        %
        % R = R1 * R2 is a robot object that is equivalent to mounting robot R2 
        % on the end of robot R1.
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
        % - this method is invoked implicitly at the command line when the result 
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
        %SerialLink.char String representation of parametesrs
        %
        % S = R.char() is a string representation of the robot parameters.

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
                s = strvcat(s, line);

                % link parameters
                s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');
                s = strvcat(s, '| j |     theta |         d |         a |     alpha |');
                s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');
                s = strvcat(s, char(r.links, true));
                s = strvcat(s, '+---+-----------+-----------+-----------+-----------+');

                % gravity, base, tool
                s_grav = horzcat(strvcat('grav = ', ' ', ' '), mat2str(r.gravity));
                s_grav = strvcat(s_grav, ' ');
                s_base = horzcat(strvcat('  base = ',' ',' ', ' '), mat2str(r.base));

                s_tool = horzcat(strvcat('   tool =  ',' ',' ', ' '), mat2str(r.tool));

                line = horzcat(s_grav, s_base, s_tool);

                s = strvcat(s, ' ', line);
                if j ~= length(robot)
                    s = strvcat(s, ' ');
                end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %   set/get methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


        function r = set.tool(r, v)
            if isempty(v)
                r.base = eye(4,4);
            elseif ~ishomog(v)
                error('tool must be a homogeneous transform');
            else
                r.tool = v;
            end
        end

        function r = set.base(r, v)
            if isempty(v)
                r.base = eye(4,4);
            elseif ~ishomog(v)
                error('base must be a homogeneous transform');
            else
                r.base = v;
            end
        end

        function r = set.offset(r, v)
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

        function r = set.qlim(r, v)
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
            v = [r.links.qlim];
        end

        function r = set.gravity(r, v)
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
        % V = R.ISLIMIT(Q) is a vector of boolean values, one per joint, 
        % false (0) if Q(i) is within the joint limits, else true (1).
            L = r.links;
            if length(q) ~= r.n
                error('argument for islimit method is wrong length');
            end
            v = [];
            for i=1:r.n
                v = [v; r.links(i).islimit(q(i))];
            end
        end

        function v = isspherical(r)
        %SerialLink.isspherical Test for spherical wrist
        %
        % R.isspherical() is true if the robot has a spherical wrist, that is, the 
        % last 3 axes intersect at a point.
        %
        % See also SerialLink.ikine6s.
            L = r.links(end-2:end);

            v = false;
            if ~isempty( find( [L(1).a L(2).a L(3).a L(2).d L(3).d] ~= 0 ))
                return
            end

            if (abs(L(1).alpha) == pi/2) & (abs(L(1).alpha + L(2).alpha) < eps)
                v = true;
                return;
            end
        end
        
        function payload(r, m, p)
        %SerialLink.payload Add payload to end of manipulator
        %
        % R.payload(M, P) adds a payload with point mass M at position P 
        % in the end-effector coordinate frame.
        %
        % See also SerialLink.ikine6s.
            lastlink = r.links(r.n);
            lastlink.m = m;
            lastlink.r = p;
        end
        
        function jt = jtraj(r, T1, T2, t, varargin)
        %SerialLink.jtraj Create joint space trajectory
        %
        % Q = R.jtraj(T0, TF, M) is a joint space trajectory where the joint
        % coordinates reflect motion from end-effector pose T0 to TF in M steps  with 
        % default zero boundary conditions for velocity and acceleration.  
        % The trajectory Q is an MxN matrix, with one row per time step, and 
        % one column per joint, where N is the number of robot joints.
        %
        % Note::
        % - requires solution of inverse kinematics. R.ikine6s() is used if
        %   appropriate, else R.ikine().  Additional trailing arguments to R.jtraj()
        %   are passed as trailing arugments to the these functions.
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


        function dyn(r)
        %SerialLink.dyn Display inertial properties
        %
        % R.dyn() displays the inertial properties of the SerialLink object in a multi-line 
        % format.  The properties shown are mass, centre of mass, inertia, gear ratio, 
        % motor inertia and motor friction.
        %
        % See also Link.dyn.
            for j=1:r.n
                fprintf('----- link %d\n', j);
                r.links(j).dyn()
            end
        end

        function record(r)
            r.qteach = [r.qteach; r.q];
        end
    end % methods

end % classdef

% utility function
function s = mat2str(m)
    m(abs(m)<eps) = 0;
    s = num2str(m);
end

