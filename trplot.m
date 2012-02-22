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
% TRPLOT(R, OPTIONS) draws a 3D coordinate frame represented by the orthonormal
% rotation matrix R (3x3).
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
% 'frame',F          The frame is named {F} and the subscript on the axis labels is F.
% 'text_opts', opt   A cell array of MATLAB text properties
% 'handle',H         Draw in the MATLAB axes specified by the axis handle H
% 'view',V           Set plot view parameters V=[az el] angles, or 'auto' 
%                    for view toward origin of coordinate frame
% 'arrow'            Use arrows rather than line segments for the axes
% 'width', w         Width of arrow tips
%
% Examples::
%
%       trplot(T, 'frame', 'A')
%       trplot(T, 'frame', 'A', 'color', 'b')
%       trplot(T1, 'frame', 'A', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
%
%       h = trplot(T, 'frame', 'A', 'color', 'b');
%       trplot(h, T2);
%
% Notes::
% - The arrow option requires the third party package arrow3.
% - The handle H is an hgtransform object.
% - When using the form TRPLOT(H, ...) the axes are not rescaled.
%
% See also TRPLOT2, TRANIMATE.


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
        return;
    end

    if size(T,3) > 1
        error('trplot cannot operate on a sequence');
    end
    if ~ishomog(T) && ~isrot(T)
        error('trplot operates only on transform (4x4) or rotation matrix (3x3)');
    end
    
    opt.color = 'b';
    opt.axes = true;
    opt.axis = [];
    opt.frame = [];
    opt.text_opts = [];
    opt.view = [];
    opt.width = 1;
    opt.arrow = false;
    opt.handle = [];

    opt = tb_optparse(opt, varargin);

    if isempty(opt.text_opts)
        opt.text_opts = {};
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

    opt.text_opts = {opt.text_opts{:}, 'Color', opt.color};

    hg = hgtransform('Parent', hax);

    % trplot( Q.R, fmt, color);
    if isrot(T)
        T = r2t(T);
    end

    % create unit vectors
    o =  [0 0 0]';
    x1 = [1 0 0]';
    y1 = [0 1 0]';
    z1 = [0 0 1]';
    
    % draw the axes
    
    mstart = [o o o]';
    mend = [x1 y1 z1]';

    if opt.arrow
        % draw the 3 arrows
        S = [opt.color num2str(opt.width)];
        ha = arrow3(mstart, mend, S);
        for h=ha'
            set(h, 'Parent', hg);
        end
    else
        for i=1:3
            h = plot2([mstart(i,1:3); mend(i,1:3)], 'Color', opt.color, 'Parent', hg);
        end
    end
    
    % label the axes
    if isempty(opt.frame)
        fmt = '%c';
    else
        fmt = sprintf('%%c_{%s}', opt.frame);
    end
    
    % add the labels to each axis
    h = text(x1(1), x1(2), x1(3), sprintf(fmt, 'X'), 'Parent', hg);
    set(h, opt.text_opts{:});
   
    h = text(y1(1), y1(2), y1(3), sprintf(fmt, 'Y'), 'Parent', hg);
    set(h, opt.text_opts{:});

    h = text(z1(1), z1(2), z1(3), sprintf(fmt, 'Z'), 'Parent', hg);
    set(h, opt.text_opts{:});
    
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
    if isstr(opt.view) && strcmp(opt.view, 'auto')
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

    % optionally return the handle, for later modification of pose
    if nargout > 0
        hout = hg;
    end
