%SerialLink.teach Graphical teach pendant
%
% R.teach() drive a graphical robot by means of a graphical slider panel.
% If no graphical robot exists one is created in a new window.  Otherwise
% all current instanes of the graphical robots are driven.
%
% R.teach(Q) specifies the initial joint angle, otherwise it is taken from 
% one of the existing graphical robots.
%
% See also SerialLink.plot.


% Ryan Steindl based on Robotics Toolbox for MATLAB (v6 and v9)
%
% Copyright (C) 1993-2011, by Peter I. Corke
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

function teach(r, varargin)
    bgcol = [135 206 250]/255;

    opt.degrees = false;
    opt.q0 = [];
% TODO: options for rpy, or eul angle display

    opt = tb_optparse(opt, varargin);

    % drivebot(r, q)
    % drivebot(r, 'deg')
    scale = ones(r.n,1);

    n = r.n;
    width = 300;
    height = 40;
    minVal = -pi;
    maxVal = pi;    

    qlim = r.qlim;
    if isempty(qlim)
        qlim = [minVal*ones(r.n,1) maxVal*ones(r.n,1)];
    end

    if isempty(opt.q0)
        q = zeros(1,n);
    else
        q = opt.q0;
    end

    % set up scale factor
    scale = [];
    for L=r.links
        if opt.degrees && L.revolute
            scale = [scale 180/pi];
        else
            scale = [scale 1];
        end
    end

    T6 = r.fkine(q);
    fig = figure('Units', 'pixels', ...
        'Position', [0 -height width height*(n+2)+20], ...
        'Color', bgcol);
    set(fig,'MenuBar','none')
    delete( get(fig, 'Children') )

    % first we check to see if there are any graphical robots of
    % this name, if so we use them, otherwise create a robot plot.

    rh = findobj('Tag', r.name);

    % attempt to get current joint config of graphical robot
    if ~isempty(rh)
        rr = get(rh(1), 'UserData');
        if ~isempty(rr.q)
            q = rr.q;
        end
    end

    % now make the sliders
    for i=1:n
        % slider label
        uicontrol(fig, 'Style', 'text', ...
            'Units', 'pixels', ...
            'BackgroundColor', bgcol, ...
            'Position', [0 height*(n-i)+20 width*0.1 height*0.4], ...
            'String', sprintf('q%d', i));

        % slider itself
        q(i) = max( qlim(i,1), min( qlim(i,2), q(i) ) ); % clip to range
        h(i) = uicontrol(fig, 'Style', 'slider', ...
            'Units', 'pixels', ...
            'Position', [width*0.1 height*(n-i)+20 width*0.7 height*0.4], ...
            'Min', scale(i)*qlim(i,1), ...
            'Max', scale(i)*qlim(i,2), ...
            'Value', scale(i)*q(i), ...
            'Tag', sprintf('Slider%d', i), ...
            'Callback', @(src,event)teach_callback(r.name, i));

        % if findjobj exists use it, since it lets us get continous callbacks while
        % a slider moves
        if exist('findjobj') && ~ispc
            drawnow
            jh = findjobj(h(i),'nomenu');
            jh.AdjustmentValueChangedCallback = {@sliderCallbackFunc, r.name, i};
        end
        % text box showing slider value, also editable
        h2(i) = uicontrol(fig, 'Style', 'edit', ...
            'Units', 'pixels', ...
            'Position', [width*0.8 height*(n-i-0.1)+20 width*0.2 height*0.7], ...
            'String', num2str(scale(i)*q(i)), ...
            'Tag', sprintf('Edit%d', i), ...
            'Callback', @(src,event)teach_callback(r.name, i));

        % hang handles off the slider and edit objects
        handles = {h(i) h2(i) scale};
        set(h(i), 'Userdata', handles);
        set(h2(i), 'Userdata', handles);
    end

    % robot name text box
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'FontSize', 20, ...
        'HorizontalAlignment', 'left', ...
        'Position', [0 height*(n+1.2)+20 0.8*width 0.8*height], ...
        'BackgroundColor', 'white', ...
        'String', r.name);

    % X
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0 height*(n+0.5)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'x:');

    h3(1,1) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.06*width height*(n+0.5)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(1,4)), ...
        'Tag', 'T6');

    % Y
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.26*width height*(n+0.5)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'y:');

    h3(2,1) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.32*width height*(n+0.5)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(2,4)));

    % Z
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.52*width height*(n+0.5)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'z:');

    h3(3,1) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.58*width height*(n+0.5)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(3,4)));

    % AX
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0 height*(n)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'ax:');

    h3(1,2) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.06*width height*(n)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(1,3)));

    % AY
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.26*width height*(n)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'ay:');

    h3(2,2) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.32*width height*(n)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(2,3)));

    % AZ
    uicontrol(fig, 'Style', 'text', ...
        'Units', 'pixels', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.52*width height*(n)+20 0.06*width height/2], ...
        'BackgroundColor', 'yellow', ...
        'FontSize', 10, ...
        'HorizontalAlignment', 'right', ...
        'String', 'az:');

    h3(3,2) = uicontrol(fig, 'Style', 'edit', ...
        'Units', 'pixels', ...
        'Position', [0.58*width height*(n)+20 width*0.2 height*0.6], ...
        'String', sprintf('%.3f', T6(3,3)));


    set(h3(1,1), 'Userdata', h3);
    uicontrol(fig, 'Style', 'pushbutton', ...
        'Units', 'pixels', ...
        'FontSize', 16, ...
        'Position', [0.8*width height*n+20 0.2*width 2*height], ...
        'CallBack', 'delete(gcf)', ...
        'BackgroundColor', 'red', ...
        'String', 'Quit');


    if isempty(rh)
        figure
        r.plot(q);
    end
end
        
function teach_callback(a, b)
% called on changes to a slider or to the edit box showing joint coordinate
% teach_callback(robot name, joint index)

    name = a; % name of the robot
    j = b;    % joint index

    % find all graphical objects tagged with the robot name, this is the 
    % instancs of that robot across all figures
    rh = findobj('Tag', name);

    % get the handles {slider, textbox, scale factor}
    handles = get(gcbo, 'Userdata');
    scale = handles{3};

    for r=rh'       % for every graphical robot instance
        robot = get(r, 'UserData'); % get the robot object
        q = robot.q;    % get its current joint angles
        if isempty(q)
            q = zeros(1,robot.n);
        end

        if gcbo == handles{1}
            % get value from slider
            q(j) = get(gcbo, 'Value') / scale(j);
            set(handles{2}, 'String', num2str(scale(j)*q(j)));
        elseif gcbo == handles{2}
            % get value from text box
            q(j) = str2num(get(gcbo, 'String')) / scale(j);
            set(handles{1}, 'Value', q(j));
        else
            warning('shouldnt happen');
        end
        robot.q = q;
        set(r, 'UserData', robot);
        robot.plot(q)
    end

    % compute and display the T6 pose
    T6 = robot.fkine(q);
    h3 = get(findobj('Tag', 'T6'), 'UserData');
    for i=1:3
        set(h3(i,1), 'String', sprintf('%.3f', T6(i,4)));
        set(h3(i,2), 'String', sprintf('%.3f', T6(i,3)));
    end

    robot.T = T6;
    robot.notify('Moved');
    drawnow
end
    
function sliderCallbackFunc(src, ev, name, joint)
    if get(src,'ValueIsAdjusting') == 1
        try
            teach_callback(name, joint);
            drawnow
        catch
            return
        end
    end
end
