%ETS2 Elementary transform sequence in 2D
%
% This class and package allows experimentation with sequences of spatial
% transformations in 2D.
%
%          import +ETS2.*
%          a1 = 1; a2 = 1;
%          E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2)
%
% Operation methods::
%   fkine
%
% Information methods::
%   isjoint    test if transform is a joint
%   njoints    the number of joint variables
%   structure  a string listing the joint types
%
% Display methods::
%   display    display value as a string
%   plot       graphically display the sequence as a robot
%   teach      graphically display as robot and allow user control
%
% Conversion methods::
%   char       convert to string
%   string     convert to string with symbolic variables
%
% Operators::
%   *          compound two elementary transforms
%   +          compound two elementary transforms
% Notes::
% - The sequence is an array of objects of superclass ETS2, but with
%   distinct subclasses: Rz, Tx, Ty.
%
%
% See also ETS3.
classdef ETS2
    properties
        what    % string name of transform: Rz, Tx, Ty
        param   % the constant parameter, if not 'qN'
        qvar    % the joint index if a joint, N if 'qN' is given, else NaN
        limits  % for prismatic joint, a 2 vector [min,max]
    end
    
    methods
        function obj = ETS2(what, x, varargin)
            %ETS2.ETS2  Create an ETS2 object
            %
            % E = ETS(W, V) is a new ETS2 object that defines an elementary transform where
            % W is 'Rz', 'Tx' or 'Ty' and V is the paramter for the transform.  If V is a string
            % of the form 'qN' where N is an integer then the transform is considered
            % to be a joint.  Otherwise the transform is a constant.
            %
            % E = ETS2(E1) is a new ETS2 object that is a clone of the ETS2 object E1.
            %
            % See also ETS2.Rz, ETS2.Tx, ETS2.Ty.
            
            assert(nargin > 0, 'RTB:ETS2:ETS2:badarg', 'no arguments given');
            
            obj.qvar = NaN;
            obj.param = 0;
            
            if nargin > 1
                if isa(x, 'ETS2')
                    % clone it
                    obj.what = x.what;
                    obj.qvar = x.qvar;
                    obj.param = x.param;
                else
                    % create a new one
                    if ischar(x)
                        obj.qvar = str2num(x(2:end));
                    else
                        obj.param = x;
                    end
                    obj.what = what;
                end
                %             else
                %                 obj.what = what;
            end
        end
        
        function r = fkine(ets, q, varargin)
            %ETS2.fkine Forward kinematics
            %
            % ETS.fkine(Q, OPTIONS) is the forward kinematics, the pose of the end of the
            % sequence as an SE2 object.  Q (1xN) is a vector of joint variables.
            %
            % ETS.fkine(Q, N, OPTIONS) as above but process only the first N elements
            % of the transform sequence.
            %
            % Options::
            %  'deg'     Angles are given in degrees.
            r = SE2;
            
            opt.deg = false;
            [opt,args] = tb_optparse(opt, varargin);
            
            if opt.deg
                opt.deg = pi/180;
            else
                opt.deg = 1;
            end
            
            n = length(ets);
            if ~isempty(args) && isreal(args{1})
                n = args{1};
            end
            assert(n>0 && n <= length(ets), 'RTB:ETS2:badarg', 'bad value of n given');
            
            for i=1:n
                e = ets(i);
                if e.isjoint
                    v = q(e.qvar);
                else
                    v = e.param;
                end
                switch e.what
                    case 'Tx'
                        r = r * SE2(v, 0, 0);
                    case 'Ty'
                        r = r * SE2(0, v, 0);
                    case 'Rz'
                        r = r * SE2(0, 0, v*opt.deg);
                end
            end
            r = r.simplify();  % simplify it if symbolic
        end
        
        function b = isjoint(ets)
            %ETS2.isjoint  Test if transform is a joint
            %
            % E.isjoint is true if the transform element is a joint, that is, its
            % parameter is of the form 'qN'.
            b = ~isnan(ets.qvar);
        end
        
        
        function k = find(ets, j)
            %ETS2.find  Find joints in transform sequence
            %
            % E.find(J) is the index in the transform sequence ETS (1xN) corresponding
            % to the J'th joint.
            [~,k] = find([ets.qvar] == j);
        end
        
        function n = njoints(ets)
            %ETS2.njoints  Number of joints in transform sequence
            %
            % E.njoints is the number of joints in the transform sequence.
            n = max([ets.qvar]);
        end
        
        function v = n(ets)
            v = ets.njoints;
        end
        
        
        function s = string(ets)
            %ETS2.string  Convert to string with symbolic variables
            %
            % E.string is a string representation of the transform sequence where
            % non-joint parameters have symbolic names L1, L2, L3 etc.
            %
            % See also trchain.
            for i = 1:length(ets)
                e = ets(i);
                if e.isjoint
                    term = sprintf('%s(q%d)', e.what, e.qvar);
                else
                    term = sprintf('%s(L%d)', e.what, constant);
                    constant = constant + 1;
                end
                if i == 1
                    s = term;
                else
                    s = [s ' ' term];
                end
            end
        end
        
        
        
        function out = mtimes(ets1, ets2)
            %ETS2.mtimes Compound transforms
            %
            % E1 * E2 is a sequence of two elementary transform.
            %
            % See also ETS2.plus.
            out = [ets1 ets2];
        end
        
        function out = plus(ets1, ets2)
            %ETS2.plus Compound transforms
            %
            % E1 + E2 is a sequence of two elementary transform.
            %
            % See also ETS2.mtimes.
            out = [ets1 ets2];
        end
        
        
        function s = structure(ets)
            %ETS2.structure  Show joint type structure
            %
            % E.structure is a character array comprising the letters 'R' or 'P' that
            % indicates the types of joints in the elementary transform sequence E.
            %
            % Notes::
            % - The string will be E.njoints long.
            %
            % See also SerialLink.config.
            s = '';
            for e = ets
                if e.qvar > 0
                    switch e.what
                        case {'Tx', 'Ty'}
                            s = [s 'P'];
                        case 'Rz'
                            s = [s 'R'];
                    end
                end
            end
        end
        
        function display(ets)
            %ETS2.display Display parameters
            %
            % E.display() displays the transform or transform sequence parameters in
            % compact single line format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is an ETS2 object and the command has no trailing
            %   semicolon.
            %
            % See also ETS2.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(ets) );
        end % display()
        
        
        function v = isprismatic(ets)
            v = isjoint(ets) && (ets.what(1) == 'T');
        end
        
        function s = char(ets)
            %ETS2.char Convert to string
            %
            % E.char() is a string showing transform parameters in a compact format.  If E is a transform sequence (1xN) then
            % the string describes each element in sequence in a single line format.
            %
            % See also ETS2.display.
            s = '';
            
            function s = render(z)
                if isa(z, 'sym')
                    s = char(z);
                else
                    s = sprintf('%g', z);
                end
            end
            
            for e = ets
                if e.isjoint
                    s = [s sprintf('%s(q%d)', e.what, e.qvar) ];
                else
                    s = [s sprintf('%s(%s)', e.what, render(e.param))];
                    
                end
            end
        end
        
        function teach(robot, varargin)
            
            %-------------------------------
            % parameters for teach panel
            bgcol = [135 206 250]/255;  % background color
            height = 0.06;  % height of slider rows
            %-------------------------------
            
            
            %---- handle options
            opt.deg = true;
            opt.orientation = {'rpy', 'eul', 'approach'};
            opt.d_2d = true;
            opt.callback = [];
            [opt,args] = tb_optparse(opt, varargin);
            
            RTBPlot.install_teach_panel('ETS2', robot, [0 0], opt);
        end
        
        
        function plot(ets, qq, varargin)
            
            % heuristic to figure robot size
            opt = RTBPlot.plot_options([], [varargin 'reach', 3, 'top'])
            h = draw_ets(ets, qq, opt);
            
            set(gca, 'Tag', 'RTB.plot');
            set(gcf, 'Units', 'Normalized');
            pf = get(gcf, 'Position');
            
            if opt.raise
                % note this is a very time consuming operation
                figure(gcf);
            end
            
            if strcmp(opt.projection, 'perspective')
                set(gca, 'Projection', 'perspective');
            end
            
            if isstr(opt.view)
                switch opt.view
                    case 'top'
                        view(0, 90);
                    case 'x'
                        view(0, 0);
                    case 'y'
                        view(90, 0)
                    otherwise
                        error('rtb:plot:badarg', 'view must be: x, y, top')
                end
            elseif isnumeric(opt.view) && length(opt.view) == 2
                view(opt.view)
            end
            
            % enable mouse-based 3D rotation
            rotate3d on
            
            ets.animate(qq);
        end
        
        function animate(ets, q)
            handles = findobj('Tag', 'ETS2');
            h = handles.UserData;
            opt = h.opt;
            
            ets = h.ets;
            
            for i=1:length(ets)
                e = ets(i);
                if i == 1
                    T = SE2;
                else
                    T = ets.fkine(q, i-1, 'setopt', opt);
                end
                
                % create the transform for displaying this element (joint cylinder + link)
                set(h.element(i), 'Matrix', T.SE3.T);
                if isprismatic(e)
                    switch e.what
                        case 'Tx'
                            set(h.pjoint(e.qvar), 'Matrix', diag([q(e.qvar) 1 1 1]));
                        case 'Ty'
                            set(h.pjoint(e.qvar), 'Matrix', diag([1 q(e.qvar) 1 1]));
                    end
                end
            end
            
            T = ets.fkine(q, 'setopt', opt);
            % animate the wrist frame
            if ~isempty(h.wrist)
                trplot2(T, 'handle', h.wrist);
            end
            
        end
        
        %----------------------------------------------------------
        
        function h_ = draw_ets(ets, q, opt)
            
            clf
            disp('creating new ETS plot');
            
            
            axis(opt.workspace);
            
            s = opt.scale;
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                axis(opt.workspace);
                hold on
            end
            
            group = hggroup('Tag', 'ETS2');
            h.group = group;
            
            % create the graphical joint and link elements
            for i=1:length(ets)
                e = ets(i);
                
                if opt.debug
                    fprintf('create graphics for %s\n', e.char );
                end
                
                % create a graphical depiction of the transform element
                % This is drawn to resemble orthogonal plumbing.
                
                if i == 1
                    T = SE2;
                else
                    T = ets.fkine(q, i-1, 'setopt', opt);
                end
                
                % create the transform for displaying this element (joint cylinder + link)
                h.element(i) = hgtransform('Tag', sprintf('element%d', i), 'Matrix', T.SE3.T, 'Parent', h.group);
                
                if isjoint(e)
                    % it's a joint element: revolute or prismatic
                    switch e.what
                        case 'Tx'
                            h.pjoint(e.qvar) = hgtransform('Tag', 'prismatic', 'Parent', h.element(i), 'Matrix', diag([q(e.qvar) 1 1 1]));
                            RTBPlot.box('x', opt.jointdiam*s, [0 1], opt.pjointcolor, [], 'Parent', h.pjoint(i));
                        case 'Ty'
                            h.pjoint(e.qvar) = hgtransform('Tag', 'prismatic', 'Parent', h.element(i), 'Matrix', diag([1 q(e.qvar) 1 1]));
                            RTBPlot.box('y', opt.jointdiam*s, [0 1], opt.pjointcolor, [], 'Parent', h.pjoint(i));
                        case 'Rz'
                            RTBPlot.cyl('z', opt.jointdiam*s, opt.jointlen*s*[-1 1], opt.jointcolor, [], 'Parent', h.element(i));
                    end
                else
                    % it's a constant transform
                    switch e.what
                        case 'Tx'
                            RTBPlot.cyl('x', s, [0 e.param], opt.linkcolor, [], 'Parent', h.element(i));
                        case 'Ty'
                            RTBPlot.cyl('y', s, [s e.param], opt.linkcolor, [t(1) 0 0], 'Parent', h.element(i));
                        case 'Rz'
                            % nothing to draw in this case
                    end
                end
                
                assert( ~(opt.jaxes && opt.jvec), 'RTB:ETS2:plot:badopt', 'Can''t specify ''jaxes'' and ''jvec''')
                
                % create the joint axis line
                if opt.jaxes
                    if e.isjoint
                        line('XData', [0 0], ...
                            'YData', [0 0], ...
                            'ZData', 14*s*[-1 1], ...
                            'LineStyle', ':', 'Parent', h.element(i));
                        
                        % create the joint axis label
                        text(0, 0, 14*s, sprintf('q%d', e.qvar), 'Parent', h.element(i))
                    end
                end
                
                % create the joint axis vector
                if opt.jvec
                    if e.isjoint
                        daspect([1 1 1]);
                        ha = arrow3([0 0 -12*s], [0 0 15*s], 'c');
                        set(ha, 'Parent', h.element(i));
                        
                        % create the joint axis label
                        text(0, 0, 20*s, sprintf('q%d', e.qvar), 'Parent', h.element(i))
                    end
                end
                
            end
            
            % display the wrist coordinate frame
            if opt.wrist
                if opt.arrow
                    % compute arrow3 scale factor...
                    d = axis(gca);
                    if length(d) == 4
                        d = norm( d(3:4)-d(1:2) ) / 72;
                    else
                        d = norm( d(4:6)-d(1:3) ) / 72;
                    end
                    extra = {'arrow', 'width', 1.5*s/d};
                else
                    extra = {};
                end
                h.wrist = trplot2(eye(3,3), 'labels', upper(opt.wristlabel), ...
                    'color', 'k', 'length', opt.wristlen*s, extra{:});
            else
                h.wrist = [];
            end
            
            xlabel('X')
            ylabel('Y')
            zlabel('Z')
            grid on
            
            % restore hold setting
            if ~ish
                hold off
            end
            
            h.opt = opt;
            h.ets = ets;
            
            if nargout > 0
                h_ = h;
            end
            
            % attach the handle structure to the top graphical element
            
            h.q = q;
            handles.opt = opt;
            
            set(group, 'UserData', h);
        end
        
        
    end
end