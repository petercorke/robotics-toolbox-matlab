%TRPLOT Draw a coordinate frame
%
% TRPLOT(T, OPTIONS) draws a 3D coordinate frame represented by the homogeneous 
% transform T (4x4).
%
% H = TRPLOT(T, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, T) moves the coordinate frame described by the handle H to
% the pose T (4x4).
%
% TRPLOT(R, OPTIONS) as above but the coordinate frame is rotated about the
% origin according to the orthonormal rotation matrix R (3x3).
%
% H = TRPLOT(R, OPTIONS) as above but returns a handle.
%
% TRPLOT(H, R) moves the coordinate frame described by the handle H to
% the orientation R.
%
% Options::
% 'color',C          The color to draw the axes, MATLAB colorspec C
% 'noaxes'           Don't display axes on the plot
% 'axis',A           Set dimensions of the MATLAB axes to A=[xmin xmax ymin ymax zmin zmax]
% 'frame',F          The coordinate frame is named {F} and the subscript on the axis labels is F.
% 'text_opts', opt   A cell array of MATLAB text properties
% 'handle',H         Draw in the MATLAB axes specified by the axis handle H
% 'view',V           Set plot view parameters V=[az el] angles, or 'auto' 
%                    for view toward origin of coordinate frame
% 'length',s         Length of the coordinate frame arms (default 1)
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips (default 1)
% 'thick',t          Thickness of lines (default 0.5)
% '3d'               Plot in 3D using anaglyph graphics
% 'anaglyph',A       Specify anaglyph colors for '3d' as 2 characters for 
%                    left and right (default colors 'rc'): chosen from
%                    r)ed, g)reen, b)lue, c)yan, m)agenta.
% 'dispar',D         Disparity for 3d display (default 0.1)
% 'text'             Enable display of X,Y,Z labels on the frame
% 'labels',L         Label the X,Y,Z axes with the 1st, 2nd, 3rd character of the string L
% 'rgb'              Display X,Y,Z axes in colors red, green, blue respectively
% 'rviz'             Display chunky rviz style axes
%
% Examples::
%
%       trplot(T, 'frame', 'A')
%       trplot(T, 'frame', 'A', 'color', 'b')
%       trplot(T1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
%       trplot(T1, 'labels', 'NOA');
%
%       h = trplot(T, 'frame', 'A', 'color', 'b');
%       trplot(h, T2);
%
% 3D anaglyph plot
%       trplot(T, '3d');
%
% Notes::
% - The 'rviz' option is equivalent to 'rgb', 'notext', 'noarrow', 
%   'thick', 5.
% - The arrow option requires the third party package arrow3 from File
%   Exchange.
% - The handle H is an hgtransform object. 
% - When using the form TRPLOT(H, ...) to animate a frame it is best to set 
%   the axis bounds.
% - The '3d' option requires that the plot is viewed with anaglyph glasses.
% - You cannot specify 'color' and '3d' at the same time.
%
% See also TRPLOT2, TRANIMATE.

%TODO:
% 'rviz', chunky RGB lines, no arrows

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
%  need to decide how to handle scaling
%  what does hold on mean?  don't touch scaling?

function hout = trplot(T, varargin)

    if isscalar(T) && ishandle(T)
        % trplot(H, T)
        H = T; T = varargin{1};
        if isrot(T)
            T = r2t(T);
        end
        set(H, 'Matrix', T);
        
        % for the 3D case retrieve the right hgtransform and set it
        hg2 = get(H, 'UserData');
        if ~isempty(hg2)
            set(hg2, 'Matrix', T);
        end
        
        return;
    end

    if size(T,3) > 1
        error('trplot cannot operate on a sequence');
    end
    if ~ishomog(T) && ~isrot(T)
        error('trplot operates only on transform (4x4) or rotation matrix (3x3)');
    end
    
    opt.color = [];
    opt.rgb = false;
    opt.axes = true;
    opt.axis = [];
    opt.frame = [];
    opt.text_opts = [];
    opt.view = [];
    opt.width = 1;
    opt.arrow = false;
    opt.labels = 'XYZ';
    opt.handle = [];
    opt.anaglyph = 'rc';
    opt.d_3d = false;
    opt.dispar = 0.1;
    opt.thick = 0.5;
    opt.length = 1;
    opt.text = true;
    opt.lefty = false;
    opt.rviz = false;

    opt = tb_optparse(opt, varargin);
        
    if opt.rviz
        opt.thick = 5;
        opt.arrow = false;
        opt.rgb = true;
        opt.text = false;
    end

    if ~isempty(opt.color) && opt.d_3d
        error('cannot specify ''color'' and ''3d'', use ''anaglyph'' option');
    end
    if isempty(opt.color)
        opt.color = 'b';
    end
    if isempty(opt.text_opts)
        opt.text_opts = {};
    end
    
    if opt.d_3d
        opt.color = ag_color(opt.anaglyph(1));
    end
    
    if isempty(opt.axis)
        % determine some default axis dimensions
        
        % get the origin of the frame
        if isrot(T)
            c = [0 0 0];  % at zero for a rotation matrix
        else
            c = transl(T);    
        end
        
        d = 1.2;
        opt.axis = [c(1)-d c(1)+d c(2)-d c(2)+d c(3)-d c(3)+d];
        
    end
    
    % TODO: should do the 2D case as well
    
    if ~isempty(opt.handle)
        hax = opt.handle;
        hold(hax);
    else
        ih = ishold;
        if ~ih
            % if hold is not on, then clear the axes and set scaling
            cla
            if ~isempty(opt.axis)
                axis(opt.axis);
            end
            daspect([1 1 1]);
            
            if opt.axes
                xlabel( 'X');
                ylabel( 'Y');
                zlabel( 'Z');
                rotate3d on
            end
            new_plot = true;
        end
        hax = gca;
        hold on
    end
    % hax is the handle for the axis we will work with, either new or
    % passed by option 'handle'

    opt.text_opts = [opt.text_opts, 'Color', opt.color];


    hg = hgtransform('Parent', hax);


    % trplot( Q.R, fmt, color);
    if isrot(T)
        T = r2t(T);
    end

    % create unit vectors
    o =  [0 0 0]';
    x1 = opt.length*[1 0 0]';
    y1 = opt.length*[0 1 0]';
    if opt.lefty
        z1 = opt.length*[0 0 -1]';
    else
        z1 = opt.length*[0 0 1]';
    end
    
    % draw the axes
    
    mstart = [o o o]';
    mend = [x1 y1 z1]';

    if opt.rgb
        axcolors = {'r', 'g', 'b'};
    else
        axcolors = { opt.color, opt.color, opt.color};
    end
    
    if opt.arrow
%         % draw the 3 arrows
%         S = [opt.color num2str(opt.width)];
%         ha = arrow3(mstart, mend, S);
%         for h=ha'
%             set(h, 'Parent', hg);
%         end
          daspect([1,1,1])
          for i=1:3
              ha = arrow3(mstart(i,1:3), mend(i,1:3), [axcolors{i} num2str(opt.width)]);
              set(ha, 'Parent', hg);
          end
    else
        for i=1:3
            plot2([mstart(i,1:3); mend(i,1:3)], 'Color', axcolors{i}, ...
                'LineWidth', opt.thick, ...
                'Parent', hg);
        end
    end
    
    % label the axes
    if isempty(opt.frame)
        fmt = '%c';
    else
        fmt = sprintf('%%c_{%s}', opt.frame);
    end
    
    if opt.text
        % add the labels to each axis
        h = text(x1(1), x1(2), x1(3), sprintf(fmt, opt.labels(1)), 'Parent', hg);
        set(h, opt.text_opts{:});
        
        h = text(y1(1), y1(2), y1(3), sprintf(fmt, opt.labels(2)), 'Parent', hg);
        set(h, opt.text_opts{:});
        
        h = text(z1(1), z1(2), z1(3), sprintf(fmt, opt.labels(3)), 'Parent', hg);
        set(h, opt.text_opts{:});
    end
    
    % label the frame
    if ~isempty(opt.frame)
        h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), o(3)-0.04*z1(3), ...
            ['\{' opt.frame '\}'], 'Parent', hg);
        set(h, 'VerticalAlignment', 'middle', ...
            'HorizontalAlignment', 'center', opt.text_opts{:});
    end
    
    if ~opt.axes
        set(gca, 'visible', 'off');
    end
    if ischar(opt.view) && strcmp(opt.view, 'auto')
        cam = x1+y1+z1;
        view(cam(1:3));
    elseif ~isempty(opt.view)
        view(opt.view);
    end
    if isempty(opt.handle) && ~ih
        grid on
        hold off
    end
    
    % now place the frame in the desired pose
    set(hg, 'Matrix', T);

    
    if opt.d_3d
        % 3D display.  The original axes are for the left eye, and we add 
        % another set of axes to the figure for the right eye view and
        % displace its camera to the right of that of that for the left eye.
        % Then we recursively call trplot() to create the right eye view.
        
        left = gca;
        right = axes;
        
        % compute the offset in world coordinates
        off = -t2r(view(left))'*[opt.dispar 0 0]';
        pos = get(left, 'CameraPosition');
        
        set(right, 'CameraPosition', pos+off');
        set(right, 'CameraViewAngle', get(left, 'CameraViewAngle'));
        set(right, 'CameraUpVector', get(left, 'CameraUpVector'));
        target = get(left, 'CameraTarget');
        set(right, 'CameraTarget', target+off');
        
        % set perspective projections
                set(left, 'Projection', 'perspective');
        set(right, 'Projection', 'perspective');
        
        % turn off axes for right view
        set(right, 'Visible', 'Off');
        
        % set color for right view
        hg2 = trplot(T, 'color', ag_color(opt.anaglyph(2)));
        
        % the hgtransform for the right view is user data for the left
        % view hgtransform, we need to change both when we rotate the 
        % frame.
        set(hg, 'UserData', hg2);
    end

    % optionally return the handle, for later modification of pose
    if nargout > 0
        hout = hg;
    end
end

function out = ag_color(c)

% map color character to an color triple, same as anaglyph.m

    % map single letter color codes to image planes
    switch c
    case 'r'
        out = [1 0 0];        % red
    case 'g'
        out = [0 1 0];        % green
    case 'b'
        % blue
        out = [0 0 1];
    case 'c'
        out = [0 1 1];        % cyan
    case 'm'
        out = [1 0 1];        % magenta
    case 'o'
        out = [1 1 0];        % orange
    end
end
