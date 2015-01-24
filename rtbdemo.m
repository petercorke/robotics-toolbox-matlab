%RTBDEMO 	Robot toolbox demonstrations
%
% rtbdemo displays a menu of toolbox demonstration scripts that illustrate:
%   - homogeneous transformations
%   - trajectories
%   - forward kinematics
%   - inverse kinematics
%   - robot animation
%   - inverse dynamics
%   - forward dynamics
%
% rtbdemo(T) as above but waits for T seconds after every statement, no
% need to push the enter key periodically.
%
% Notes::
% - By default the scripts require the user to periodically hit <Enter> in
%   order to move through the explanation.


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

function rtbdemo(timeout)
    echo off
    close all
    
    % find the path to the demos
    if exist('rtbdemo', 'file') == 2
        tbpath = fileparts(which('rtbdemo'));
        demopath = fullfile(tbpath, 'demos');
    end
    
    % create the options to pass through to runscript
    opts = {'begin', 'path', demopath};
    
    % if a timeout interval is given, add this to the options
    if nargin > 0
        opts = {opts, 'delay', timeout};
    end
    
    % display a help message in the consolde
    msg = {
'------------------------------------------------------------'
'Many of these demos print tutorial text and MATLAB commmands'
'in the console window.  Read the text and press <enter> to move'
'on to the next command. At the end of the tutorial you can' 
'choose the next one from the graphical menu, or close the menu'
'window.'
'------------------------------------------------------------'
};
    for i=1:numel(msg)
        fprintf('%s\n', msg{i});
    end
    
    % Map the button names (must be exact match) to the scripts to invoke
    demos = {
        'Rotations', 'rotation';
        'Transformations', 'trans';
        'Joystick demo', 'joytest';
        'Trajectory', 'traj';
        'V-REP simulator', 'vrepdemo';
        'Create a model', 'robot';
        'Animation', 'graphics';
        'Forward kinematics', 'fkine';
        'Inverse kinematics', 'ikine';
        'Jacobians', 'jacob';
        'Inverse dynamics', 'idyn';
        'Forward dynamics', 'fdyn';
        'Symbolic', 'symbolic';
        'Code generation', 'codegen';
        'Driving to a pose', 'drivepose';
        'Quadrotor flying', 'quadrotor';
        'Braitenberg vehicle', 'braitnav';
        'Bug navigation', 'bugnav';
        'D* navigation', 'dstarnav';
        'PRM navigation', 'prmnav';
        'SLAM demo', 'slam';
        'Particle filter localization', 'particlefilt';
        };
    
    % display the GUI panel
    %  some of this taken from the GUIDE generated file
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       'rtbdemo_gui', ...
        'gui_Singleton',  gui_Singleton, ...
        'gui_OpeningFcn', @rtbdemo_gui_OpeningFcn, ...
        'gui_OutputFcn',  @rtbdemo_gui_OutputFcn, ...
        'gui_LayoutFcn',  [] , ...
        'gui_Callback',   []);
    h = gui_mainfcn(gui_State);

    
    % now set the callback for every button, can't seem to make this work using
    % GUIDE.
    for hh=get(h, 'Children')'
        if ~strcmp( get(hh, 'Style'), 'pushbutton')
            continue;
        end
        set(hh, 'Callback', @demo_pushbutton_callback);
    end
    
    % TODO:
    %  build the buttons dynamically, eliminate the need for GUIDE
    
    set(h, 'Name', 'rtbdemo');
    
    while true
        
        set(h, 'Visible', 'on');


        % wait for a button press
        %   buttons set the UserData property of the GUI to the button string name
        waitfor(h, 'UserData');
        
        % check if the GUI window has been dismissed
        if ~ishandle(h)
            break
        end
        
        % get the user's selection, it was stashed in GUI UserData
        selection = get(h, 'UserData');
        set(h, 'UserData', []); % reset user data so we notice next change
        
        % now look for it in the list of demos
        for i=1:size(demos, 1)
            if strcmp(selection, demos{i,1})
                % then run the appropriate script
                script = demos{i,2}
                set(h, 'Visible', 'off');
                try
                runscript(script, opts{:})
                catch me
                    disp('error in executing demo script');
                    me.getReport()
                end
            end
        end
    end
end

% --- Executes just before rtbdemo_gui is made visible.
function rtbdemo_gui_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to rtbdemo_gui (see VARARGIN)
    
    % Choose default command line output for rtbdemo_gui
    handles.output = hObject;
    
    % Update handles structure
    guidata(hObject, handles);
    
    initialize_gui(hObject, handles, false);
end

% --- Outputs from this function are returned to the command line.
function varargout = rtbdemo_gui_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    
    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

function initialize_gui(fig_handle, handles, isreset)
    % Update handles structure
    guidata(handles.figure1, handles);
end

% --- Executes on button press .
function demo_pushbutton_callback(hObject, eventdata, handles)
        % hObject    handle to pushbutton (see GCBO)
        % eventdata  reserved - to be defined in a future version of MATLAB
        % handles    structure with handles and user data (see GUIDATA)
        set(gcf, 'Userdata', get(hObject, 'String'));
    end
