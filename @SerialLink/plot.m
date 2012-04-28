%SerialLink.ploT Graphical display and animation
%
% R.plot(Q, options) displays a graphical animation of a robot based on 
% the kinematic model.  A stick figure polyline joins the origins of
% the link coordinate frames. The robot is displayed at the joint angle Q, or 
% if a matrix it is animated as the robot moves along the trajectory.
%
% The graphical robot object holds a copy of the robot object and
% the graphical element is tagged with the robot's name (.name property).
% This state also holds the last joint configuration which can be retrieved,
% see PLOT(robot) below.
%
% Figure behaviour::
% If no robot of this name is currently displayed then a robot will
% be drawn in the current figure.  If hold is enabled (hold on) then the
% robot will be added to the current figure.
%
% If the robot already exists then that graphical model will be found 
% and moved.
%
% Multiple views of the same robot::
%
% If one or more plots of this robot already exist then these will all
% be moved according to the argument Q.  All robots in all windows with 
% the same name will be moved.
%
% Multiple robots in the same figure::
%
% Multiple robots can be displayed in the same plot, by using "hold on"
% before calls to plot(robot).  
%
% Graphical robot state::
%
% The configuration of the robot as displayed is stored in the SerialLink object
% and can be accessed by the read only object property R.q.
%
% Graphical annotations and options::
%
% The robot is displayed as a basic stick figure robot with annotations 
% such as:
% - shadow on the floor
% - XYZ wrist axes and labels
% - joint cylinders and axes
% which are controlled by options.
%
% The size of the annotations is determined using a simple heuristic from 
% the workspace dimensions.  This dimension can be changed by setting the 
% multiplicative scale factor using the 'mag' option.
%
% Options::
%  'workspace', W          size of robot 3D workspace, W = [xmn, xmx ymn ymx zmn zmx]
%  'delay', d              delay betwen frames for animation (s)
%  'fps',fps               set number of frames per second for display
%  'cylinder', C           color for joint cylinders, C=[r g b]
%  'mag', scale            annotation scale factor
%  'perspective'|'ortho'   type of camera view
%  'raise'|'noraise'       controls autoraise of current figure on plot, is
%                          incredibly slow.
%  'render'|'norender'     controls shaded rendering after drawing
%  'loop'|'noloop'         controls endless loop mode
%  'base'|'nobase'         controls display of base 'pedestal'
%  'wrist'|'nowrist'       controls display of wrist
%  'shadow'|'noshadow'     controls display of shadow
%  'name'|'noname'         display the robot's name 
%  'xyz'|'noa'             wrist axis label
%  'jaxes'|'nojaxes'       control display of joint axes
%  'joints'|'nojoints'     controls display of joints
%
% The options come from 3 sources and are processed in order:
% - Cell array of options returned by the function PLOTBOTOPT.
% - Cell array of options given by the 'plotopt' option when creating the
%   SerialLink object.
% - List of arguments in the command line.
%
% See also plotbotopt, SerialLink.fkine.


% HANDLES:
%
%  A robot comprises a bunch of individual graphical elements and these are 
% kept in a structure which can be stored within the .handle element of a
% robot object:
%   h.robot     the robot stick figure
%   h.shadow    the robot's shadow
%   h.x     wrist vectors
%   h.y
%   h.z
%   h.xt        wrist vector labels
%   h.yt
%   h.zt
%
%  The plot function returns a new robot object with the handle element set.
%
% For the h.robot object we additionally: 
%   - save this new robot object as its UserData
%   - tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.



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

function retval = plot(robot, tg, varargin)

    % opts = PLOT(robot, options)
    %  just convert options list to an options struct
    if (nargin == 2) && iscell(tg)
        retval = plot_options(robot, varargin{:});
        return;
    end
        
    %
    % q = PLOT(robot)
    % return joint coordinates from a graphical robot of given name
    %
    %TODO should be robot.get_q()
    if nargin == 1
        handles = findobj('Tag', robot.name);
        if ~isempty(handles)
            h = get(handles(1), 'UserData');
            retval = h.q;
        end
        return
    end
    
    % process options
    if (nargin > 2) && isstruct(varargin{1})
        % options is a struct
        opt = varargin{1};
    else
        % options is a list of options
        opt = plot_options(robot, varargin);
    end

    %
    % robot2 = ROBOT(robot, q, varargin)
    %
    np = numrows(tg);
    n = robot.n;

    if numcols(tg) ~= n
        error('Insufficient columns in q')
    end

%   if ~ishandle(robot.handle),
%        %disp('has handles')
%       % handles provided, animate just that robot
%        count = opt.repeat;
%        while count > 0
%           for p=1:np,
%                animate( robot, tg(p,:));
%                pause(opt.delay)
%           end
%            count = count - 1;
%       end
%       return;
%    end

    % get handle of any existing graphical robot of same name
    handles = findobj('Tag', robot.name);

    if isempty(handles) || isempty( get(gcf, 'Children'))
        % no robot of this name exists

        % create one
        newplot();
        handle = create_new_robot(robot, opt);

        % tag one of the graphical handles with the robot name and hang
        % the handle structure off it
        set(handle.links, 'Tag', robot.name);
        set(handle.links, 'UserData', handle);

        handles = handle.links;
    end

    if ishold && isempty( findobj(gca, 'Tag', robot.name))
        % if hold is on and no robot of this name in current axes
        h = create_new_robot(robot, opt);

        % tag one of the graphical handles with the robot name and hang
        % the handle structure off it
        set(handle.links, 'Tag', robot.name);
        set(handle.links, 'UserData', handle);

        handles = handle.links;
    end
    
    if opt.raise
        % note this is a very time consuming operation
        figure(gcf);
    end

    while true
        for p=1:np      % for each point on path
            robot.animate(tg(p,:), handles);
            if opt.delay > 0
                pause(opt.delay);
            end
        end
        if ~opt.loop
            break;
        end
    end

    % save the last joint angles away in the graphical robot
    for handle=handles'
        h = get(handle, 'UserData');
        h.q = tg(end,:);
        set(handle, 'UserData', h);
    end


%PLOT_OPTIONS
%
%   o = PLOT_OPTIONS(robot, options)
%
% Returns an options structure

function o = plot_options(robot, optin)
    % process a cell array of options and return a struct

    % define all possible options and their default values
    o.erasemode = 'normal';
    o.joints = true;
    o.wrist = true;
    o.loop = false;
    o.shadow = true;
    o.wrist = true;
    o.jaxes = true;
    o.base = true;
    o.wristlabel = 'xyz';
    o.perspective = true;
    o.magscale = 1;
    o.name = true;
    o.delay = 0.1;
    o.raise = true;
    o.cylinder = [0 0 0.7];
    o.workspace = [];

    % build a list of options from all sources
    %   1. the M-file plotbotopt if it exists
    %   2. robot.plotopt
    %   3. command line arguments
    if exist('plotbotopt', 'file') == 2
        options = [plotbotopt robot.plotopt optin];
    else
        options = [robot.plotopt optin];
    end

    % parse the options
    [o,args] = tb_optparse(o, options);
    if ~isempty(args)
        error(['unknown option: ' args{1}]);
    end

    if isempty(o.workspace)
        %
        % simple heuristic to figure the maximum reach of the robot
        %
        L = robot.links;
        reach = 0;
        for i=1:robot.n
            reach = reach + abs(L(i).a) + abs(L(i).d);
        end
        o.workspace = [-reach reach -reach reach -reach reach];
        o.mag = reach/10;
    else
        reach = min(abs(o.workspace));
    end
    o.mag = o.magscale * reach/10;

%CREATE_NEW_ROBOT
% 
%   h = CREATE_NEW_ROBOT(robot, opt)
%
% Using data from robot object and options create a graphical robot in
% the current figure.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%   

% h.mag
% h.zmin
% h.links   the line segment that is the robot
% h.shadow  the robot's shadow
% h.x       the line segment that is T6 x-axis
% h.y       the line segment that is T6 x-axis
% h.z       the line segment that is T6 x-axis
% h.xt      text for T6 x-axis
% h.yt      text for T6 y-axis
% h.zt      text for T6 z-axis
% h.joint(i)        the joint i cylinder or box
% h.jointaxis(i)    the line segment that is joint i axis
% h.jointlabel(i)   text for joint i label

function h = create_new_robot(robot, opt)
    h.opt = opt;            % the options
    h.robot = robot;        % pointer to robot

    h.mag = opt.mag;
    %
    % setup an axis in which to animate the robot
    %
    % handles not provided, create graphics
    %disp('in creat_new_robot')
    if ~ishold
        % if current figure has hold on, then draw robot here
        % otherwise, create a new figure
        axis(opt.workspace);
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    %set(gca, 'drawmode', 'fast');
    grid on

    zlim = get(gca, 'ZLim');
    h.zmin = zlim(1);

    if opt.base
        b = transl(robot.base);
        line('xdata', [b(1);b(1)], ...
            'ydata', [b(2);b(2)], ...
            'zdata', [h.zmin;b(3)], ...
            'LineWidth', 4, ...
            'color', 'red');
    end
    
    if opt.name
        b = transl(robot.base);
        text(b(1), b(2)-opt.mag, [' ' robot.name], 'FontAngle', 'italic', 'FontWeight', 'bold')
    end
    % create a line which we will
    % subsequently modify.  Set erase mode to xor for fast
    % update
    %
    h.links = line(robot.lineopt{:});
    
    if opt.shadow
        h.shadow = line(robot.shadowopt{:}, ...
            'Erasemode', opt.erasemode);
    end

    if opt.wrist,   
        h.x = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'red');
        h.y = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'green');
        h.z = line('xdata', [0;0], ...
            'ydata', [0;0], ...
            'zdata', [0;0], ...
            'color', 'blue');
        h.xt = text(0, 0, opt.wristlabel(1), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
        h.yt = text(0, 0, opt.wristlabel(2), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');
        h.zt = text(0, 0, opt.wristlabel(3), 'FontWeight', 'bold', 'HorizontalAlignment', 'Center');

    end

    %
    % display cylinders (revolute) or boxes (pristmatic) at
    % each joint, as well as axis centerline.
    %
    L = robot.links;
    for i=1:robot.n
        
        if opt.joints

            % cylinder or box to represent the joint
            if L(i).sigma == 0
                N = 16;
            else
                N = 4;
            end
            % define the vertices of the cylinder
            [xc,yc,zc] = cylinder(opt.mag/4, N);
            zc(zc==0) = -opt.mag/2;
            zc(zc==1) = opt.mag/2;

            % create vertex color data
            cdata = zeros(size(xc));
            for j=1:3
                cdata(:,:,j) = opt.cylinder(j);
            end
            % render the surface
            h.joint(i) = surface(xc,yc,zc,cdata);
            
            % set the surfaces to be smoothed and translucent
            set(h.joint(i), 'FaceColor', 'interp');
            set(h.joint(i), 'EdgeColor', 'none');
            set(h.joint(i), 'FaceAlpha', 0.7);

            % build a matrix of coordinates so we
            % can transform the cylinder in animate()
            % and hang it off the cylinder
            xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,2*N+2)]; 
            set(h.joint(i), 'UserData', xyz);
        end

        if opt.jaxes
            % add a dashed line along the axis
            h.jointaxis(i) = line('xdata', [0;0], ...
                'ydata', [0;0], ...
                'zdata', [0;0], ...
                'color', 'blue', ...
                'linestyle', ':');
            h.jointlabel(i) = text(0, 0, 0, num2str(i), 'HorizontalAlignment', 'Center');
        end
    end
