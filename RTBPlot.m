%RTBPlot Plot utilities for Robotics Toolbox

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
classdef RTBPlot
    
    methods (Static)
        
        function th = install_teach_panel(name, robot, q, opt)
            %
            % robot like object, has n fkine animate methods
            
            %-------------------------------
            % parameters for teach panel
            bgcol = [135 206 250]/255;  % background color
            height = 0.06;  % height of slider rows
            %-------------------------------
            
            
            %---- install the panel at the side of the figure
            
            % find the right figure to put it in
            c = findobj(gca, 'Tag', name);  % check the current axes
            if isempty(c)
                % doesn't exist in current axes, look wider
                c = findobj(0, 'Tag', name);  % check all figures
                if ~isempty(c)
                    ax = get(c(1), 'Parent'); % get first axis holding the robot
                else
                    error('RTB:RTBPlot:install_teach_panel', 'no window found');
                end
            else
                % found it in current axes
                ax = gca;
            end
            teachhandles.fig = get(ax, 'Parent');  % get the figure that holds the axis
            
            % shrink the current axes to make room
            %   [l b w h]
            
            if opt.d_2d
                ax.ZColor = 'none';
                ax.Color = 'none';
                ax.Position = [0.22 0.05 0.8 1];
                
            else
                ax.Position = [0.3 0 0.7 1];
            end
            
            teachhandles.curax = ax;
            
            % create the panel itself
            panel = uipanel(teachhandles.fig, ...
                'Title', 'Teach', ...
                'BackGroundColor', bgcol,...
                'Position', [0 0 0.25 1]);
            panel.Units = 'pixels'; % stop automatic resizing
            teachhandles.panel = panel;
            set(teachhandles.fig, 'Units', 'pixels');
            
            set(teachhandles.fig, 'ResizeFcn', @(src,event) RTBPlot.resize_callback(robot, teachhandles));
            
            
            %---- get the current robot state
            
            %             if isempty(q)
            %                 % check to see if there are any graphical robots of this name
            %                 rhandles = findobj('Tag', robot.name);
            %
            %                 % find the graphical element of this name
            %                 assert(~isempty(rhandles), 'RTB:teach:badarg', 'No graphical robot of this name found');
            %
            %                 % get the info from its Userdata
            %                 info = get(rhandles(1), 'UserData');
            %
            %                 % the handle contains current joint angles (set by plot)
            %                 if ~isempty(info.q)
            %                     q = info.q;
            %                 end
            %             else
            %                 robot.plot(q);
            %             end
            teachhandles.q = q;
            T6 = robot.fkine(q);
            
            if isa(T6, 'SE2')
                T6 = T6.SE3;
            end
            T6 = T6.T;
            
            % we need to have qlim set to finite values for a prismatic joint
            if isa(robot, 'SerialLink')
                qlim = robot.qlim;
                assert(~any(isinf(qlim(:))), 'RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
                
                % set up scale factor, from actual limits in radians/metres to display units
                qscale = ones(robot.n,1);
                for j=1:robot.n
                    L=robot.links(j);
                    if opt.deg && L.isrevolute
                        qscale(j) = 180/pi;
                    end
                end
                
            else
                % for an ETS*
                for i=1:robot.n
                    if robot(i).isprismatic
                        qlim(i,:) = [0 2*robot(i).param];
                    else
                        qlim(i,:) = pi*[-1 1];
                    end
                end
                
                % set up scale factor, from actual limits in radians/metres to display units
                qscale = ones(robot.n,1);
                for j=1:robot.n
                    if ~robot(i).isprismatic && opt.deg
                        qscale(j) = 180/pi;
                    else
                        qscale(j) = 1;
                    end
                end
            end
            
            teachhandles.qscale = qscale;
            teachhandles.robot = robot;
            teachhandles.q = q;
            teachhandles.orientation = opt.orientation;
            teachhandles.opt = opt;
            
            %---- now make the sliders
            n = robot.n;
            for j=1:n
                % slider label
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0 height*(n-j+2) 0.15 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String', sprintf('q%d', j));
                
                % slider itself
                q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
                teachhandles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.15 height*(n-j+2) 0.65 height], ...
                    'Min', qlim(j,1), ...
                    'Max', qlim(j,2), ...
                    'Value', q(j), ...
                    'TooltipString', sprintf('Joint %d value', j), ...
                    'Tag', sprintf('Slider%d', j));
                
                % text box showing slider value, also editable
                teachhandles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.80 height*(n-j+2)+.01 0.20 0.9*height], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(qscale(j)*q(j), 3), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4, ...
                    'Tag', sprintf('Edit%d', j));
            end
            
            %---- set up the position display box
            
            % X
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', 'x:');
            
            teachhandles.t6.t(1) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(1,4)), ...
                'TooltipString', 'End-effector x-coordinate', ...
                'Tag', 'T6');
            
            % Y
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-2*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', 'y:');
            
            teachhandles.t6.t(2) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-2*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'TooltipString', 'End-effector y-coordinate', ...
                'String', sprintf('%.3f', T6(2,4)));
            
            if ~opt.d_2d
                % Z
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 1-3*height 0.2 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.9, ...
                    'HorizontalAlignment', 'left', ...
                    'String', 'z:');
                
                teachhandles.t6.t(3) = uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'Position', [0.3 1-3*height 0.6 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.8, ...
                    'TooltipString', 'End-effector z-coordinate', ...
                    'String', sprintf('%.3f', T6(3,4)));
            end
            
            % Orientation
            switch opt.orientation
                case 'approach'
                    labels = {'ax:', 'ay:', 'az:'};
                    tips = {'Approach vector - x component', 'Approach vector - y component', 'Approach vector - z component'};
                case 'eul'
                    labels = {[char(hex2dec('3c6')) ':'], [char(hex2dec('3b8')) ':'], [char(hex2dec('3c8')) ':']}; % phi theta psi
                    tips = {'Euler angle phi (about Z)', 'Euler angle theta (about Y)', 'Euler angle psi (about Z)'};
                case {'rpy', 'rpy/xyz'}
                    labels = {'R:', 'P:', 'Y:'};
                    tips = {'Roll angle (about Z)', 'Pitch angle (about Y)', 'Yaw angle (about X)'};
                case 'rpy/zyx'
                    labels = {'R:', 'P:', 'Y:'};
                    tips = {'Roll angle (about X)', 'Pitch angle (about Y)', 'Yaw angle (about Z)'};
            end
            
            %---- set up the orientation display box
            
            if opt.d_2d
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 1-5*height 0.2 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.9, ...
                    'HorizontalAlignment', 'left', ...
                    'String', 'Yaw:');
                
                teachhandles.t6.r(1) = uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'Position', [0.3 1-5*height 0.6 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.8, ...
                    'TooltipString', 'Yaw angle (about Z)', ...
                    'String', sprintf('%.3f', 0));
            else
                % AX
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 1-5*height 0.2 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.9, ...
                    'HorizontalAlignment', 'left', ...
                    'String', labels(1));
                
                teachhandles.t6.r(1) = uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'Position', [0.3 1-5*height 0.6 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.8, ...
                    'TooltipString', tips{1}, ...
                    'String', sprintf('%.3f', 0));
                
                % AY
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 1-6*height 0.2 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.9, ...
                    'HorizontalAlignment', 'left', ...
                    'String', labels(2));
                
                teachhandles.t6.r(2) = uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'Position', [0.3 1-6*height 0.6 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.8, ...
                    'TooltipString', tips{2}, ...
                    'String', sprintf('%.3f', 0));
                
                % AZ
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 1-7*height 0.2 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.9, ...
                    'HorizontalAlignment', 'left', ...
                    'String', labels(3));
                
                teachhandles.t6.r(3) = uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'Position', [0.3 1-7*height 0.6 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.8, ...
                    'TooltipString', tips{3}, ...
                    'String', sprintf('%.3f', 0));
            end
            %---- add buttons
            uicontrol(panel, 'Style', 'pushbutton', ...
                'Units', 'normalized', ...
                'Position', [0.80 height*(0)+.01 0.15 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.7, ...
                'CallBack', @(src,event) RTBPlot.quit_callback(robot, teachhandles), ...
                'BackgroundColor', 'white', ...
                'ForegroundColor', 'red', ...
                'TooltipString', 'Quit', ...
                'String', 'X');
            
            % the record button
            teachhandles.record = [];
            if isfield(opt, 'record') && ~isempty(opt.record)
                uicontrol(panel, 'Style', 'pushbutton', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 height*(0)+.01 0.30 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.6, ...
                    'CallBack', @(src,event) RTBPlot.record_callback(robot, teachhandles), ...
                    'BackgroundColor', 'red', ...
                    'ForegroundColor', 'white', ...
                    'String', 'REC');
            end
            
            
            teachhandles.callback = opt.callback;
            
            
            %---- now assign the callbacks
            for j=1:n
                % text edit box
                set(teachhandles.edit(j), ...
                    'Interruptible', 'off', ...
                    'Callback', @(src,event)RTBPlot.teach_callback(src, name, j, teachhandles));
                
                % slider
                set(teachhandles.slider(j), ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue' );
                
                % ask for continuous callbacks
                addlistener(teachhandles.slider(j), 'ContinuousValueChange', ...
                    @(src,event)RTBPlot.teach_callback(src, name, j, teachhandles) );
                
            end
            
            % refresh the display
            RTBPlot.teach_callback([], name, [], teachhandles);
            
            if nargout > 0
                th = teachhandles;
            end
        end
        
        function teach_callback(src, name, j, teachhandles)
            
            % called on changes to a slider or to the edit box showing joint coordinate
            %
            % src      the object that caused the event
            % name     name of the robot
            % j        the joint index concerned (1..N)
            % slider   true if the
            
            qscale = teachhandles.qscale;
            
            if ~isempty(src)
                switch get(src, 'Style')
                    case 'slider'
                        % slider changed, get value and reflect it to edit box
                        newval = get(src, 'Value');
                        set(teachhandles.edit(j), 'String', num2str(qscale(j)*newval, 3));
                    case 'edit'
                        % edit box changed, get value and reflect it to slider
                        newval = str2double(get(src, 'String')) / qscale(j);
                        set(teachhandles.slider(j), 'Value', newval);
                end
            end
            %fprintf('newval %d %f\n', j, newval);
            
            
            
            % find all graphical objects tagged with the robot name, this is the
            % instances of that robot across all figures
            
            h = findobj('Tag', name);
            
            
            % find the graphical element of this name
            if isempty(h)
                error('RTB:teach:badarg', 'No graphical robot of this name found');
            end
            % get the info from its Userdata
            info = get(h(1), 'UserData');
            
            if ~isempty(j)
                % update the stored joint coordinates
                info.q(j) = newval;
                % and save it back to the graphical object
                set(h(1), 'UserData', info);
            end
            
            % update all robots of this name
            animate(teachhandles.robot, info.q);
            
            
            % compute the robot tool pose
            T6 = teachhandles.robot.fkine(info.q);
            if isa(T6, 'SE2')
                T6 = T6.SE3;
            end
            T6 = T6.T;
            
            % convert orientation to desired format
            switch teachhandles.orientation
                case 'approach'
                    orient = T6(:,3);    % approach vector
                case 'eul'
                    orient = tr2eul(T6, 'setopt', teachhandles.opt);
                case {'rpy','rpy/xyz'}
                    orient = tr2rpy(T6, 'xyz', 'setopt', teachhandles.opt);
                case'rpy/zyx'
                    orient = tr2rpy(T6, 'zyx', 'setopt', teachhandles.opt);
            end
            
            % update the display in the teach window
            if teachhandles.opt.d_2d
                set(teachhandles.t6.t(1), 'String', sprintf('%.3f', T6(1,4)));
                set(teachhandles.t6.t(2), 'String', sprintf('%.3f', T6(2,4)));
                
                set(teachhandles.t6.r(1), 'String', sprintf('%.3f', orient(1)));
            else
                for i=1:3
                    set(teachhandles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
                    set(teachhandles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
                end
            end
            
            if isfield(teachhandles, 'callback') && ~isempty(teachhandles.callback)
                teachhandles.callback(teachhandles.robot, info.q);
            end
            
            %notify(handles.robot, 'Moved');
            
        end
        
        function record_callback(robot, handles)
            
            if ~isempty(handles.callback)
                handles.record(h.q);
            end
        end
        
        function quit_callback(robot, handles)
            set(handles.fig, 'ResizeFcn', '');
            delete(handles.panel);
            set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
        end
        
        function resize_callback(robot, handles)
            
            % come here on figure resize events
            fig = gcbo;   % this figure (whose callback is executing)
            fs = get(fig, 'Position');  % get size of figure
            ps = get(handles.panel, 'Position');  % get position of the panel
            % update dimensions of the axis area
            set(handles.curax, 'Units', 'pixels', ...
                'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
            % keep the panel anchored to the top left corner
            set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
        end
        
        function cyl(ax, r, extent, color, offset, varargin)
            %RTBPlot.cyl Draw a cylinder
            %
            % CYL(AX, R, EXTENT, COLOR, OFFSET, OPTIONS) draws a cylinder parallel to
            % axis AX ('x', 'y' or 'z') of radius R between EXTENT(1) and EXTENT(2).
            %
            % OPTIONS are passed through to surf.
            %
            % See also surf, RTBPlot.box.
            
            n = 20;
            theta = (0:n)/n*2*pi;
            
            RTBPlot.draw_shape(ax, r, extent, color, offset, theta, varargin{:});
        end
        
        
        function box(ax, r, extent, color, offset, varargin)
            %RTBPlot.box  Draw a box
            %
            % BPX(AX, R, EXTENT, COLOR, OFFSET, OPTIONS) draws a cylinder parallel to
            % axis AX ('x', 'y' or 'z') of side length R between EXTENT(1) and EXTENT(2).
            
            theta = [1 3 5 7 9]/4*pi;
            
            RTBPlot.draw_shape(ax, r, extent, color, offset, theta, varargin{:});
        end
        
        function draw_shape(ax, r, extent, color, offset, theta, varargin)
            
            % draw nothing if extent range is zero
            if abs(extent(1) - extent(2)) < eps
                return
            end
            
            % default value for offset
            if isempty(offset)
                offset = [0 0 0];
            end
            
            r = [r;r];
            n = length(theta)-1;
            
            switch ax
                case 'x'
                    y = r * cos(theta);
                    z = r * sin(theta);
                    x = extent(:) * ones(1,n+1);
                case 'y'
                    x = r * cos(theta);
                    z = r * sin(theta);
                    y = extent(:) * ones(1,n+1);
                case 'z'
                    y = r * cos(theta);
                    x = r * sin(theta);
                    z = extent(:) * ones(1,n+1);
            end
            
            x = x + offset(1);
            y = y + offset(2);
            z = z + offset(3);
            
            % walls of the shape
            surf(x,y,z, 'FaceColor', color, 'EdgeColor', 'none', varargin{:})
            
            % put the ends on
            patch(x', y', z', color, 'EdgeColor', 'none', varargin{:});
        end
        
        function create_floor(opt)
            
            if ~isempty(opt.floorimage)
                RTBPlot.create_image_floor(opt)
            else
                RTBPlot.create_tiled_floor(opt)
            end
        end
        
        function create_image_floor(opt)
            xmin = opt.workspace(1);
            xmax = opt.workspace(2);
            ymin = opt.workspace(3);
            ymax = opt.workspace(4);

            [X,Y] = meshgrid([xmin, xmax], [ymin ymax]);
            Z = opt.floorlevel*ones(2,2);

            C = repmat(opt.floorimage, 1, 1, 3);
            surface(X, Y, Z, C, ...
                'FaceColor','texturemap',...
                'EdgeColor','none',...
                'SpecularStrength', 0, ...
                'CDataMapping','direct');
        end
        
        % draw a tiled floor in the current axes
        function create_tiled_floor(opt)
            
            if ~opt.tiles
                return
            end
            xmin = opt.workspace(1);
            xmax = opt.workspace(2);
            ymin = opt.workspace(3);
            ymax = opt.workspace(4);
            
            % create a colored tiled floor
            xt = xmin:opt.tilesize:xmax;
            yt = ymin:opt.tilesize:ymax;
            
            % check how many tiles are to be rendered and reduce it if too many
            % too many tiles can lead to MATLAB crashing with lack of memory
            n = max(length(xt), length(yt)); % number requested
            if n > 20
                n = n/20;  % reduce by this factor
                p = 10^floor(log10(n));  % figure a scale factor 1,2,5 * 10^N
                v = n/p;
                s = [1 2 5 10];
                k = find(v > s);
                f = k(end)+1;
                f = f*p;
                warning('RTB:SerialLink:plot', 'floor tiles too small, making them %f x bigger - change the size or disable them', f);
                opt.tilesize = opt.tilesize * f;
                xt = xmin:opt.tilesize:xmax;
                yt = ymin:opt.tilesize:ymax;
            end
            
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
                'SpecularStrength', 0, ...
                'CDataMapping','direct');
        end
        
        
        function opt = plot_options(robot, optin)
                        
            opt.deg = false;
            
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
            opt.reach = [];
            opt.name = true;
            opt.projection = {'ortho', 'perspective'};
            opt.view = [];
            opt.top = false;
            
            % 3D rendering
            opt.shading = true;
            opt.lightpos = [1 1 20];
            
            % tiled floor
            opt.tiles = true;
            opt.tile1color = [0.5 1 0.5];  % light green
            opt.tile2color = [1 1 1];  % white
            opt.floorlevel = [];
            opt.tilesize = 0.2;
            
            opt.floorimage = [];
            
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
            opt.wristlen = 16;
            
            % joint rotation axes
            opt.jaxes = false;
            opt.jvec = false;
            
            % joint cylinders
            opt.joints = true;
            opt.jointdiam = 1.5;
            opt.jointlen = 3;
            opt.jointcolor = [0.7 0 0];
            opt.pjointcolor = [0.4 1 0.03];
            
            % links
            opt.linkcolor = 'b';
            opt.toolcolor = 'r';
            
            % misc
            opt.movie = [];
            
            % build a list of options from all sources
            %   1. the M-file plotbotopt if it exists
            %   2. robot.plotopt
            %   3. command line arguments
            
            options = optin;
            
            if ~isempty(robot)
                options = [robot.plotopt options];
            end
            
            if exist('plotbotopt', 'file') == 2
                options = [plotbotopt options];
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
            
            if ~isempty(opt.floorimage)
                opt.tiles = false;
            end
            
            % figure the size of the figure
            
            
            if ~isempty(opt.reach)
                reach = opt.reach;
                
            else
                % reach is not specified use a simple heuristic to figure the maximum reach of the robot
                
                assert(~isempty(robot), 'RTB:RTBPlot:plot_options', 'robot must be defined to estimate reach');
                L = robot.links;
                
                reach = 0;
                for j=1:robot.n
                    if L(j).isrevolute
                        % revolute, add the link length and offset
                        reach = reach + abs(L(j).a) + abs(L(j).d);
                    else
                        % prismatic
                        if ~isempty(L(j).qlim)
                            % use the maximum joint value
                            assert(all(L(j).qlim >= 0),  'RTB:RTBPlot:plot_options', 'prismatic joint %d qlim values cannot be negative', j)
                            reach = reach + abs(L(j).a) + max(abs(L(j).qlim));
                        else
                            % prismatic joint has no maximum length provided
                            if isempty(opt.workspace)
                                % workspace was given so don't complain, but we still need to compute reach
                                %  mark it as NaN and compute it later from workspace
                                error('RTB:RTBPlot:plot_options', 'prismatic joint %d has no qlim parameter set, set it or specify ''workspace'' plot option', j);
                            else
                                reach = NaN;
                            end
                        end
                    end
                end
                reach = reach + sum(abs(robot.tool.t));  % add the tool length
            end
            
            if isnan(reach)
                % reach couldn't be estimated because a prismatic qlim was missing,
                %  estimate it from workspace
                reach = max( colnorm( reshape(opt.workspace, [2 3]) ) ) / 2;
            end
                
            if isempty(opt.workspace)
                % now create a 3D volume based on this reach
                opt.workspace = [-reach reach -reach reach -reach reach];
            end
            
            if opt.wrist
                % the wrist axes add to the maximum reach, we need to scale the wrist
                % length to keep within bounds
                f = (1 + opt.wristlen/40);
                reach = reach * f;
                opt.wristlen = 40*(1-1/f);
            end
            reach = reach/opt.zoom;
            
            % if we have a floor, quantize the reach to a tile size
            if opt.tiles
                reach = opt.tilesize * ceil(reach/opt.tilesize);
            end
            
            % figure out where the floor is
            if isempty(opt.workspace)
                
                % if a floorlevel has been given, ammend the 3D volume
                if ~isempty(opt.floorlevel)
                    opt.workspace(5) = opt.floorlevel;
                else
                    opt.floorlevel = -reach;
                end
            else
                if opt.tiles
                    % set xy limits to be integer multiple of tilesize
                    opt.workspace(1:4) = opt.tilesize * round(opt.workspace(1:4)/opt.tilesize);
                    opt.floorlevel = opt.workspace(5);
                end
            end
            
            % update the fundamental scale factor (given by the user as a multiplier) by a length derived from
            % the overall workspace dimension
            %  we need that a lot when creating the robot model
            if reach > 0
                opt.scale = opt.scale * reach/40;
            else
                opt.scale = opt.scale / 40;
            end
            
            if ~isempty(opt.fps)
                opt.delay = 1/opt.fps;
            end
            
        end
    end
end
