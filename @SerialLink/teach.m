%SerialLink.teach Graphical teach pendant
%
% R.teach(OPTIONS) drive a graphical robot by means of a graphical slider panel.
% If no graphical robot exists one is created in a new window.  Otherwise
% all current instances of the graphical robot are driven.
%
% h = R.teach(OPTIONS) as above but returns a handle for the teach window.  Can
% be used to programmatically delete the teach window.
%
% Options::
% 'eul'        Display tool orientation in Euler angles
% 'rpy'        Display tool orientation in roll/pitch/yaw angles
% 'approach'   Display tool orientation as approach vector (z-axis)
% 'degrees'    Display angles in degrees (default radians)
% 'q0',q       Set initial joint coordinates
%
% GUI::
%
% - The record button adds the current joint coordinates as a row to the robot's
%   qteach property.
% - The Quit button destroys the teach window.
%
% Notes::
% - The slider limits are derived from the joint limit properties.  If not
%   set then for
%   - a revolute joint they are assumed to be [-pi, +pi]
%   - a prismatic joint they are assumed unknown and an error occurs.
%
% See also SerialLink.plot.

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

%% TODO:
%%  make the sliders change the animation while moving
%% http://www.mathworks.com/matlabcentral/newsreader/view_thread/159374
%% 1. download FINDJOBJ from the file exchange: http://www.mathworks.com/matlabcentral/fileexchange/14317
%% 2. hSlider = uicontrol('style','slider', ...); %create the slider, get its Matlab handle
%% 3. jSlider = findjobj(hSlider,'nomenu'); %get handle of the underlying java object
%% 4. jSlider.AdjustmentValueChangedCallback = @myMatlabFunction; %set callback
%% 
%% Note: you can also use the familiar format:
%% set(jSlider,'AdjustmentValueChangedCallback',@myMatlabFunction)
%% 
%% Feel free to explore many other properties (and ~30 callbacks)
%% available in jSlider but not in its Matlab front-end interface hSlider.
%% 
%% Warning: FINDJOBJ relies on undocumented and unsupported Matlab/Java
%% functionality.

function handle = teach(r, varargin)
    bgcol = [135 206 250]/255;

    opt.degrees = false;
    opt.q0 = [];
    opt.orientation = {'approach', 'eul', 'rpy'};
% TODO: options for rpy, or eul angle display

    opt = tb_optparse(opt, varargin);

    % drivebot(r, q)
    % drivebot(r, 'deg')

    n = r.n;
    width = 300;
    height = 40;

    qlim = r.qlim;
    if any(isinf(qlim))
        error('for prismatic axes must define joint coordinate limits, set qlim properties for prismatic Links');
    end

    if isempty(opt.q0)
        q = zeros(1,n);
    else
        q = opt.q0;
    end

    % set up scale factor, from actual limits in radians/metres to display units
    qscale = ones(r.n,1);
    for i=1:r.n
        L=r.links(i);
        if opt.degrees && L.isrevolute
            qscale(i) = 180/pi;
        end
    end
    
    handles.qscale = qscale;
    handles.orientation = opt.orientation;

    T6 = r.fkine(q);

    fig = figure('Units', 'pixels', ...
        'Position', [0 -height width height*(n+2)+40], ...
        'BusyAction', 'cancel', ...
        'HandleVisibility', 'off', ...
        'Color', bgcol);
    set(fig,'MenuBar','none')
    delete( get(fig, 'Children') )

    % first we check to see if there are any graphical robots of
    % this name, if so we use them, otherwise create a robot plot.

    rhandles = findobj('Tag', r.name);

    % attempt to get current joint config of graphical robot
    if ~isempty(rhandles)
        h = get(rhandles(1), 'UserData');
        if ~isempty(h.q)
            q = h.q;
        end
    end

    % now make the sliders
    for i=1:n
        % slider label
        uicontrol(fig, 'Style', 'text', ...
            'Units', 'pixels', ...
            'BackgroundColor', bgcol, ...
            'Position', [0 height*(n-i)+40 width*0.1 height*0.4], ...
            'String', sprintf('q%d', i));

        % slider itself
        q(i) = max( qlim(i,1), min( qlim(i,2), q(i) ) ); % clip to range
        handles.slider(i) = uicontrol(fig, 'Style', 'slider', ...
            'Units', 'pixels', ...
            'Position', [width*0.1 height*(n-i)+40 width*0.7 height*0.4], ...
            'Min', qlim(i,1), ...
            'Max', qlim(i,2), ...
            'Value', q(i), ...
            'Tag', sprintf('Slider%d', i));

        % text box showing slider value, also editable
        handles.edit(i) = uicontrol(fig, 'Style', 'edit', ...
            'Units', 'pixels', ...
            'Position', [width*0.8 height*(n-i-0.1)+40 width*0.2 height*0.7], ...
            'String', num2str(qscale(i)*q(i)), ...
            'Tag', sprintf('Edit%d', i));
    end

    set(handles.slider(1))

        
    % robot name text box
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'FontSize', 20, ...
        'HorizontalAlignment', 'left', ...
        'Position', [0 height*(n+1.2)+40 0.8*width 0.8*height], ...
        'BackgroundColor', 'white', ...
        'String', r.name);

    % X
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0 height*(n+0.5)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'x:');

    handles.t6.t(1) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.06*width height*(n+0.5)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(1,4)), ...
        'Tag', 'T6');

    % Y
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.26*width height*(n+0.5)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'y:');

    handles.t6.t(2) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.32*width height*(n+0.5)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(2,4)));

    % Z
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.52*width height*(n+0.5)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'z:');

    handles.t6.t(3) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.58*width height*(n+0.5)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(3,4)));

    % Orientation
    switch opt.orientation
        case 'approach'
            labels = {'ax:', 'ay:', 'az:'};
        case 'eul'
            labels = {'\phi:', '\theta:', '\psi:'};
        case'rpy'
            labels = {'R:', 'P:', 'Y:'};
    end
    % AX
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0 height*(n)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', labels(1));

    handles.t6.r(1) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.06*width height*(n)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(1,3)));

    % AY
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.26*width height*(n)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', labels(2));

    handles.t6.r(2) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.32*width height*(n)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(2,3)));

    % AZ
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.52*width height*(n)+40 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', labels(2));

    handles.t6.r(3) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.58*width height*(n)+40 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(3,3)));

    % add buttons
    uicontrol(fig, 'Style', 'pushbutton', ...
        'Units', 'pixels', ...
        'FontSize', 16, ...
        'Position', [0.8*width height*n+40 0.2*width 2*height], ...
        'CallBack', @(src,event) delete(fig), ...
        'BackgroundColor', 'red', ...
        'String', 'Quit');
    % the record button
    handles.record = [];
    uicontrol(fig, 'Style', 'pushbutton', ...
        'Units', 'pixels', ...
        'FontSize', 15, ...
        'Position', [0.05*width 8 0.9*width 0.7*height], ...
        'CallBack', @(src,event) record_callback(r, handles), ...
        'BackgroundColor', 'blue', ...
        'ForegroundColor', 'white', ...
        'String', 'record');
    

    % now assign the callbacks
    for i=1:n
        % text edit box
        set(handles.edit(i), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)teach_callback(src, r.name, i, handles));
        
        % slider
        set(handles.slider(i), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)teach_callback(src, r.name, i, handles));

        % if findjobj exists use it, since it lets us get continous callbacks while
        % a slider moves
        if (exist('findjobj', 'file')>0)  && verLessThan('matlab', '8') && ~ispc
            disp('using findjobj');
            pause(0.1);
            drawnow
            jh = findjobj(handles.slider(i), 'nomenu');
            %jh.AdjustmentValueChangedCallback = {@sliderCallbackFunc, r.name, i};
            jh.AdjustmentValueChangedCallback = @(src,event)sliderCallbackFunc(src, handles.slider(i), r.name, i, handles);
        end
    end

    if isempty(rhandles)
        figure
        r.plot(q);
    end
    
    if nargout > 0
        handle = fig;
    end
end

function teach_callback(src, name, j, handles)
% called on changes to a slider or to the edit box showing joint coordinate
%
% src      the object that caused the event
% name     name of the robot
% j        the joint index concerned (1..N)
% slider   true if the 

    qscale = handles.qscale;

    switch get(src, 'Style')
        case 'slider'
            % slider changed, get value and reflect it to edit box
            newval = get(src, 'Value');
            set(handles.edit(j), 'String', num2str(qscale(j)*newval));
        case 'edit'
            % edit box changed, get value and reflect it to slider
            newval = str2double(get(src, 'String')) / qscale(j);
            set(handles.slider(j), 'Value', newval);
    end
    %fprintf('newval %d %f\n', j, newval);

    
    % find all graphical objects tagged with the robot name, this is the 
    % instancs of that robot across all figures
    rhandles = findobj('Tag', name);

    for rhandle=rhandles'
        % for every graphical robot instance
        
        h = get(rhandle, 'UserData'); % get the robot object
        robot = h.robot;
        q = h.q;    % get its current joint angles
        if isempty(q)
            q = zeros(1,robot.n);
        end
        q(j) = newval;   % update the joint angle

        % update the robot object
        %robot.q = q;
        %set(r, 'UserData', robot);
        
        robot.plot(q);   % plot it
        drawnow
    end

    % compute the robot tool pose
    T6 = robot.fkine(q);
    
    % convert orientation to desired format
    switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6);
        case'rpy'
            orient = tr2rpy(T6);
    end 
 
    % update the display in the teach window
    for i=1:3
        set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
        set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
    end
    
    robot.T = T6;
    robot.notify('Moved');
    %drawnow
end

function record_callback(robot, handles)
    rhandles = findobj('Tag', robot.name);

    if ~isempty(rhandles)
        h = get(rhandles(1), 'UserData'); % get the plot graphics
        h.q
        robot.record(h.q);
    end
end
    
function sliderCallbackFunc(src, h, name, joint, handles)
    persistent busy

    % this is really ugly but it works
    if busy
        return
    end

    if get(src,'ValueIsAdjusting') == 1
        busy = true;
        try
            teach_callback(h, name, joint, handles);
        catch me
            fprintf('*******\n')
            busy = false;
            return
        end
    end
    busy = false;


end
