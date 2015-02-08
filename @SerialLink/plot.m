%SerialLink.plot Graphical display and animation
%
% R.plot(Q, options) displays a graphical animation of a robot based on
% the kinematic model.  A stick figure polyline joins the origins of
% the link coordinate frames. The robot is displayed at the joint angle Q (1xN), or
% if a matrix (MxN) it is animated as the robot moves along the M-point trajectory.
%
% Options::
% 'workspace', W    Size of robot 3D workspace, W = [xmn, xmx ymn ymx zmn zmx]
% 'floorlevel',L    Z-coordinate of floor (default -1)
%-
% 'delay',D         Delay betwen frames for animation (s)
% 'fps',fps         Number of frames per second for display, inverse of 'delay' option
% '[no]loop'        Loop over the trajectory forever
% '[no]raise'       Autoraise the figure
% 'movie',M         Save frames as files in the folder M
% 'trail',L         Draw a line recording the tip path, with line style L
%-
% 'scale',S         Annotation scale factor
% 'zoom',Z          Reduce size of auto-computed workspace by Z, makes
%                   robot look bigger
% 'ortho'           Orthographic view
% 'perspective'     Perspective view (default)
% 'view',V          Specify view V='x', 'y', 'top' or [az el] for side elevations,
%                   plan view, or general view by azimuth and elevation
%                   angle.
% 'top'             View from the top.
%-
% '[no]shading'     Enable Gouraud shading (default true)
% 'lightpos',L      Position of the light source (default [0 0 20])
% '[no]name'        Display the robot's name
%-
% '[no]wrist'       Enable display of wrist coordinate frame
% 'xyz'             Wrist axis label is XYZ
% 'noa'             Wrist axis label is NOA
% '[no]arrow'       Display wrist frame with 3D arrows
%-
% '[no]tiles'       Enable tiled floor (default true)
% 'tilesize',S      Side length of square tiles on the floor (default 0.2)
% 'tile1color',C   Color of even tiles [r g b] (default [0.5 1 0.5]  light green)
% 'tile2color',C   Color of odd tiles [r g b] (default [1 1 1] white)
%-
% '[no]shadow'      Enable display of shadow (default true)
% 'shadowcolor',C   Colorspec of shadow, [r g b]
% 'shadowwidth',W   Width of shadow line (default 6)
%-
% '[no]jaxes'       Enable display of joint axes (default false)
% '[no]jvec'        Enable display of joint axis vectors (default false)
% '[no]joints'      Enable display of joints
% 'jointcolor',C    Colorspec for joint cylinders (default [0.7 0 0])
% 'jointdiam',D     Diameter of joint cylinder in scale units (default 5)
%-
% 'linkcolor',C     Colorspec of links (default 'b')
%-
% '[no]base'        Enable display of base 'pedestal'
% 'basecolor',C     Color of base (default 'k')
% 'basewidth',W     Width of base (default 3)
%
% The options come from 3 sources and are processed in order:
% - Cell array of options returned by the function PLOTBOTOPT (if it exists)
% - Cell array of options given by the 'plotopt' option when creating the
%   SerialLink object.
% - List of arguments in the command line.
%
% Many boolean options can be enabled or disabled with the 'no' prefix.  The
% various option sources can toggle an option, the last value is taken.
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
% Figure behaviour::
%
% - If no figure exists one will be created and the robot drawn in it.
% - If no robot of this name is currently displayed then a robot will
%   be drawn in the current figure.  If hold is enabled (hold on) then the
%   robot will be added to the current figure.
% - If the robot already exists then that graphical model will be found
%   and moved.
%
% Multiple views of the same robot::
%
% If one or more plots of this robot already exist then these will all
% be moved according to the argument Q.  All robots in all windows with
% the same name will be moved.
%
%  Create a robot in figure 1
%         figure(1)
%         p560.plot(qz);
%  Create a robot in figure 2
%         figure(2)
%         p560.plot(qz);
%  Now move both robots
%         p560.plot(qn)
%
% Multiple robots in the same figure::
%
% Multiple robots can be displayed in the same plot, by using "hold on"
% before calls to robot.plot().
%
%  Create a robot in figure 1
%         figure(1)
%         p560.plot(qz);
%  Make a clone of the robot named bob
%         bob = SerialLink(p560, 'name', 'bob');
%  Draw bob in this figure
%         hold on
%         bob.plot(qn)
%
%  To animate both robots so they move together:
%         qtg = jtraj(qr, qz, 100);
%         for q=qtg'
%           p560.plot(q');
%           bob.plot(q');
%         end
%
% Making an animation movie::
% - The 'movie' options saves frames as files NNNN.png into the specified folder
% - The specified folder will be created
% - To convert frames to a movie use a command like:
%        ffmpeg -r 10 -i %04d.png out.avi
%
% Notes::
% - The options are processed when the figure is first drawn, to make different options come
%   into effect it is neccessary to clear the figure.
% - The link segments do not neccessarily represent the links of the robot, they are a pipe
%   network that joins the origins of successive link coordinate frames.
% - Delay betwen frames can be eliminated by setting option 'delay', 0 or
%   'fps', Inf.
% - By default a quite detailed plot is generated, but turning off labels,
%   axes, shadows etc. will speed things up.
% - Each graphical robot object is tagged by the robot's name and has UserData
%   that holds graphical handles and the handle of the robot object.
% - The graphical state holds the last joint configuration
% - The size of the plot volume is determined by a heuristic for an all-revolute
%   robot.  If a prismatic joint is present the 'workspace' option is
%   required.  The 'zoom' option can reduce the size of this workspace.
%
% See also SerialLink.plot3d, plotbotopt, SerialLink.animate, SerialLink.teach, SerialLink.fkine.


% HANDLES:
%
% A robot comprises a bunch of individual graphical elements and these are
% kept in a structure:
%
%   h.link     the graphical elements that comprise each joint/link
%   h.wrist     the coordinate frame marking the wrist frame
%   h.shadow    the robot's shadow
%   h.floorlevel the z-coordinate of the tiled floor
%   h.robot pointer to the robot object
%   h.opt   the final options structure
%
% The h.link graphical element is tagged with the robot's name and has this
% struct as its UserData.
%
%  h.links -> h -> robot
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.



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

% TODO
% deal with base transform and tool
% more consistent option names, scale, mag etc.
function plot(robot, qq, varargin)
    
    % check the joint angle data matches the robot
    n = robot.n;
    if numcols(qq) ~= n
        error('Insufficient columns in q')
    end
    
    % process options, these come from:
    %  - passed arguments
    %  - the robot object itself
    %  - the file plotopt.m
    opt = plot_options(robot, varargin);
    

    % logic to handle where the plot is drawn, are old figures updated or
    % replaced?
    %  calls create_floor() and create_robot() as required.
    
    if strcmp(get(gca,'Tag'), 'RTB.plot')
        % this axis is an RTB plot window
        
        rhandles = findobj('Tag', robot.name);
        
        if isempty(rhandles)
            % this robot doesnt exist here, create it or add it
            
            if ishold
                % hold is on, add the robot, don't change the floor
                handle = create_robot(robot, opt);
                
                
                % tag one of the graphical handles with the robot name and hang
                % the handle structure off it
%                 set(handle.joint(1), 'Tag', robot.name);
%                 set(handle.joint(1), 'UserData', handle);
            else
                % create the robot and floor
                newplot();

                if opt.tiles
                    create_tiled_floor(opt);
                end
                handle = create_robot(robot, opt);
                set(gca, 'Tag', 'RTB.plot');
            end
            
        end
        
    else
        % this axis never had a robot drawn in it before, let's use it
        if opt.tiles
            create_tiled_floor(opt);
        end
        handle = create_robot(robot, opt);
        set(gca, 'Tag', 'RTB.plot');
        set(gcf, 'Units', 'Normalized');
        pf = get(gcf, 'Position');
        if strcmp( get(gcf, 'WindowStyle'), 'docked') == 0
            set(gcf, 'Position', [0.1 1-pf(4) pf(3) pf(4)]);
        end
    end
    
    % deal with a few options that need to be stashed in the SerialLink object
    % movie mode has not already been flagged
    if opt.movie
        robot.framenum = 0;
        robot.moviepath = opt.movie;
    else
        robot.framenum = [];
    end
    robot.delay = opt.delay;
    robot.loop = opt.loop;   
    
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
    
    if ~isempty(opt.movie)
        mkdir(opt.movie);
        framenum = 1;
    end
    robot.animate(qq);
    
end

% Create a new graphical robot in the current figure.
% Returns a structure of handles that describe the various graphical entities in the robot model
% Make extensive use of hgtransform, all entities are defined at the origin, then moved to their
% proper pose
%
% The graphical hiearchy is:
%  hggroup: Tag = robot name
%     hgtransform: Tag = 'link#'
%
% The top-level group has user data which is the handle structure.

function h = create_robot(robot, opt)
    
    %disp('creating new robot');
    
    links = robot.links;
    s = opt.scale;
    
    % create an axis
    ish = ishold();
    if ~ishold
        % if hold is off, set the axis dimensions
        axis(opt.workspace);
        hold on
    end
    
    N = robot.n;
    
    % create the base
    if opt.base
        bt = transl(robot.base);
        bt = [bt'; bt'];
        bt(1,3) = opt.floorlevel;
        line(bt(:,1), bt(:,2), bt(:,3), 'LineWidth', opt.basewidth, 'Color', opt.basecolor);
    end
    
    % add the robot's name
    if opt.name
        b = transl(robot.base);
        bz = 0;
        if opt.base
            bz = 0.5*opt.floorlevel;
        end
        text(b(1), b(2)-s, bz, [' ' robot.name], 'FontAngle', 'italic', 'FontWeight', 'bold')
    end
    group = hggroup('Tag', robot.name);
    h.group = group;
    
    % create the graphical joint and link elements
    for L=1:N
        if opt.debug
            fprintf('create graphics for joint %d\n', L);
        end
        
        % create the transform for displaying this element (joint cylinder + link)
        h.link(L) = hgtransform('Tag', sprintf('link%d', L), 'Parent', group);
        
        % create a joint cylinder
        if opt.joints
            % create the body of the joint
            if links(L).isrevolute
                cyl('z', 2*s, opt.jointdiam/2*s*[-1 1], opt.jointcolor, [], 'Parent', h.link(L));
            else
                % create an additional hgtransform for positioning and scaling the prismatic
                % element.  The element is created with unit length.
                h.pjoint(L) = hgtransform('Tag', 'prismatic', 'Parent', h.link(L));
                if links(L).mdh
                    % make the box extend in negative z-dir because scaling factor in animate
                    % must be positive
                    box('z', s, [0 -1], opt.jointcolor, [], 'Parent', h.pjoint(L));
                else
                    box('z', s, [0 1], opt.jointcolor, [], 'Parent', h.pjoint(L));
                end
            end
        end
        
        % create the body of the link
        
        % create elements to represent translation between joint frames, ie. the link itself.
        % This is drawn to resemble orthogonal plumbing.
        if robot.mdh
            % modified DH convention
            if L < N
                A = links(L+1).A(0);
                t = transl(A);
                if t(1) ~= 0
                    cyl('x', s, [0 t(1)], opt.linkcolor, [], 'Parent', h.link(L));
                end
                if t(2) ~= 0
                    cyl('y', s, [0 t(2)], opt.linkcolor, [t(1) 0 0], 'Parent', h.link(L));
                end
                if t(3) ~= 0
                    cyl('z', s, [0 t(3)], opt.linkcolor, [t(1) t(2) 0], 'Parent', h.link(L));
                end
            end
        else
            % standard DH convention
            if L > 1
                Ainv = inv(links(L-1).A(0));
                t = transl(Ainv);
                if t(1) ~= 0
                    cyl('x', s, [0 t(1)], opt.linkcolor, [], 'Parent', h.link(L));
                end
                if t(2) ~= 0
                    cyl('y', s, [s t(2)], opt.linkcolor, [t(1) 0 0], 'Parent', h.link(L));
                end
                if t(3) ~= 0
                    cyl('z', s, [s t(3)], opt.linkcolor, [t(1) t(2) 0], 'Parent', h.link(L));
                end
                %line([0 t(1)]', [0 t(2)]', [0 t(3)]', 'Parent', h.link(L));
            end
        end
        
        if opt.jaxes && opt.jvec
            error('RTB:plot:badopt', 'Can''t specify ''jaxes'' and ''jvec''')
        end
        % create the joint axis line
        if opt.jaxes
            line('XData', [0 0], ...
                'YData', [0 0], ...
                'ZData', 12*s*[-1 1], ...
                'LineStyle', ':', 'Parent', h.link(L));
            
            % create the joint axis label
            text(0, 0, 12*s, sprintf('q%d', L), 'Parent', h.link(L))
        end
         % create the joint axis vector
        if opt.jvec
            daspect([1 1 1]);
            ha = arrow3([0 0 -12*s], [0 0 20*s]);
            set(ha, 'Parent', h.link(L));
            
            % create the joint axis label
            text(0, 0, -12*s, sprintf('q%d', L), 'Parent', h.link(L))
        end
    end
    if opt.debug
        fprintf('create graphics for tool\n');
    end
    % display the tool transform if it exists
    h.link(N+1) = hgtransform('Tag', sprintf('link%d', N+1), 'Parent', group);
    tool = eye(4,4);
    if ~robot.mdh
        tool = links(L).A(0);
    end
    if ~isempty(robot.tool)
        tool = tool * robot.tool;
    end
    t = transl(inv(tool));
    if t(1) ~= 0
        cyl('x', s, [0 t(1)], 'r', [], 'Parent', h.link(N+1));
    end
    if t(2) ~= 0
        cyl('y', s, [s t(2)], 'r', [t(1) 0 0], 'Parent', h.link(N+1));
    end
    if t(3) ~= 0
        cyl('z', s, [s t(3)], 'r', [t(1) t(2) 0], 'Parent', h.link(N+1));
    end
    
    % display the wrist coordinate frame
    if opt.wrist
        if opt.arrow
            h.wrist = trplot(eye(4,4), 'labels', upper(opt.wristlabel), ...
                'arrow', 'rgb', 'length', 15*s);
        else
            h.wrist = trplot(eye(4,4), 'labels', upper(opt.wristlabel), ...
                'rgb', 'length', 15*s);
        end
    else
        h.wrist = [];
    end
    
    % display a shadow on the floor
    if opt.shadow
        % create the polyline which is the shadow on the floor
        h.shadow = line('LineWidth', opt.shadowwidth, 'Color', opt.shadowcolor);
    end
    
    if opt.trail
        h.trail = plot(0, 0, opt.trail);
        robot.trail = [];
    end
    
    % deal with some display options
    if opt.shading
        lighting gouraud
        light('position', opt.lightpos)
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    
    % restore hold setting
    if ~ish
        hold off
    end
    h.floorlevel = opt.floorlevel;
    h.robot = robot;
    h.opt = opt;
    
    % attach the handle structure to the top graphical element
    set(group, 'UserData', h);
end

% draw a cylinder of radius r in the direction specified by ax, with an
% extent from extent(1) to extent(2)

function cyl(ax, r, extent, color, offset, varargin)
    if abs(extent(1) - extent(2)) < eps
        return
    end
    
    if isempty(offset)
        offset = [0 0 0];
    end
    
    %fprintf('   cyl: %s, r=%f, extent=[%g, %g]\n', ax, r, extent);
    
    n = 20;
    
    r = [r;r];
    
    theta = (0:n)/n*2*pi;
    sintheta = sin(theta); sintheta(n+1) = 0;
    
    switch ax
        case 'x'
            y = r * cos(theta) + offset(2);
            z = r * sintheta + offset(3);
            x = extent(:) * ones(1,n+1) + offset(1);
        case 'y'
            x = r * cos(theta) + offset(1);
            z = r * sintheta + offset(3);
            y = extent(:) * ones(1,n+1) + offset(2);
        case 'z'
            x = r * cos(theta) + offset(1);
            y = r * sintheta + offset(2);
            z = extent(:) * ones(1,n+1) + offset(3);
    end
    
    % walls of the cylinder
    surf(x,y,z, 'FaceColor', color, 'EdgeColor', 'none', varargin{:})
    
    % put the ends on
    patch(x', y', z', color, 'EdgeColor', 'none', varargin{:});
end

% draw a cylinder of radius r in the direction specified by ax, with an
% extent from extent(1) to extent(2)

function box(ax, r, extent, color, offset, varargin)
    if abs(extent(1) - extent(2)) < eps
        return
    end
    %fprintf('   box: %s, r=%f, extent=[%g, %g]\n', ax, r, extent);
    n = 4;
    
    r = [r;r];
    
    theta = (0:n)/n*2*pi;
    sintheta = sin(theta); sintheta(n+1) = 0;
    
    switch ax
        case 'x'
            y = r * cos(theta);
            z = r * sintheta;
            x = extent(:) * ones(1,n+1);
        case 'y'
            x = r * cos(theta);
            z = r * sintheta;
            y = extent(:) * ones(1,n+1);
        case 'z'
            y = r * cos(theta);
            x = r * sintheta;
            z = extent(:) * ones(1,n+1);
    end
    
    % walls of the cylinder
    surf(x,y,z, 'FaceColor', color, 'EdgeColor', 'none', varargin{:})
    
    % put the ends on
    patch(x', y', z', color, 'EdgeColor', 'none', varargin{:});
end

% draw a tiled floor in the current axes
function create_tiled_floor(opt)
    
    xmin = opt.workspace(1);
    xmax = opt.workspace(2);
    ymin = opt.workspace(3);
    ymax = opt.workspace(4);
    
    % create a colored tiled floor
    xt = xmin:opt.tilesize:xmax;
    yt = ymin:opt.tilesize:ymax;
    Z = opt.floorlevel*ones( numel(yt), numel(xt));
    C = zeros(size(Z));
    [r,c] = ind2sub(size(C), 1:numel(C));
    C = bitand(r+c,1);
    C = reshape(C, size(Z));
    C = cat(3, opt.tile1color(1)*C+opt.tile2color(1)*(1-C), ...
        opt.tile1color(2)*C+opt.tile2color(2)*(1-C), ...
        opt.tile1color(3)*C+opt.tile2color(3)*(1-C));
    [X,Y] = meshgrid(xt, yt);
    surface(X, Y, Z, C, ...
        'FaceColor','texturemap',...
        'EdgeColor','none',...
        'CDataMapping','direct');
end

    % process a cell array of options and return a struct
    % define all possible options and their default values
    
function opt = plot_options(robot, optin)
    
    % timing/looping
    opt.delay = 0.1;
    opt.fps = [];
    opt.loop = false;
    
    opt.raise = false;
    
    % general appearance
    opt.scale = 1;
    opt.zoom = 1;
    opt.trail = [];
    
    opt.workspace = [];
    opt.name = true;
    opt.projection = {'ortho', 'perspective'};
    opt.view = [];
    opt.top = false;

    % 3D rendering
    opt.shading = true;
    opt.lightpos = [0 0 20];
    
    % tiled floor
    opt.tiles = true;
    opt.tile1color = [0.5 1 0.5];  % light green
    opt.tile2color = [1 1 1];  % white
    opt.floorlevel = [];
    opt.tilesize = 0.2;
    
    % shadow on the floor
    opt.shadow = true;
    opt.shadowcolor = [0.5 0.5 0.5];
    opt.shadowwidth = 6;
     
    % the base or pedestal
    opt.base = true;
    opt.basewidth = 3;
    opt.basecolor = 'k';
    
    % wrist
    opt.wrist = true;
    opt.wristlabel = {'xyz', 'noa'};
    opt.arrow = true;
    
    % joint rotation axes
    opt.jaxes = false;
    opt.jvec = false;
    
    % joint cylinders
    opt.joints = true;
    opt.jointdiam = 5;
    opt.jointcolor = [0.7 0 0];
    
    % links
    opt.linkcolor = 'b';
    opt.toolcolor = 'r';
    
    % misc
    opt.movie = [];

    
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
    [opt,args] = tb_optparse(opt, options);
    if ~isempty(args)
        error(['unknown option: ' args{1}]);
    end
    
    if opt.top
        opt.view = 'top';
    end
    if ~isempty(opt.projection)
        opt.projection = 'ortho';
    end
    
    % figure the size of the figure
    if isempty(opt.workspace)
        %
        % simple heuristic to figure the maximum reach of the robot
        %
        L = robot.links;
        if any(L.isprismatic)
            error('Prismatic joint(s) present: requires the ''workspace'' option');
        end
        reach = 0;
        for i=1:robot.n
            reach = reach + abs(L(i).a) + abs(L(i).d);
        end
        reach = reach + sum(abs(transl(robot.tool)));
        reach = reach/opt.zoom;
        
        % if we have a floor, quantize the reach to a tile size
        if opt.tiles
            reach = opt.tilesize * ceil(reach/opt.tilesize);
        end
        
        % now create a 3D volume based on this reach
        opt.workspace = [-reach reach -reach reach -reach reach];
        
        % if a floorlevel has been given, ammend the 3D volume
        if ~isempty(opt.floorlevel)
            opt.workspace(5) = opt.floorlevel;
        else
            opt.floorlevel = -reach;
        end
    else
        reach = min(abs(diff(reshape(opt.workspace, [2 3]))));
        if opt.tiles
            % set xy limits to be integer multiple of tilesize
            opt.workspace(1:4) = opt.tilesize * round(opt.workspace(1:4)/opt.tilesize);
            opt.floorlevel = opt.workspace(5);
        end
    end
    
    % update the fundamental scale factor (given by the user as a multiplier) by a length derived from
    % the overall workspace dimension
    %  we need that a lot when creating the robot model
    opt.scale = opt.scale * reach/40;
    
    if ~isempty(opt.fps)
        opt.delay = 1/opt.fps;
    end
    
end
