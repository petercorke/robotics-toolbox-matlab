%Frame Coordinate frame object
%
% F = Frame(P, OPTIONS) creates an object that graphically renders 
% a coordinate frame for SE(2), SO(2) or SE(3) represented by the 
% pose P which can be:
% - homogeneous transform (3x3) for SE(2)
% - Quaternion for SO(3)
% - orthonormal rotation matrix (3x3) for SO(3)
% - homogeneous transform (4x4) for SE(3)
%
% Methods::
% move       move the graphical coordinate frame to a new pose
% animate    move the graphical coordinate frame to a new pose
% char
% display
% delete
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
%       f_a = Frame(TA, 'frame', 'A')
%       f_b = Frame(TB, 'frame', 'B', 'color', 'b')
%       f_c = Frame(TC, 'frame', 'C', 'text_opts', {'FontSize', 10, 'FontWeight', 'bold'})
%
%       f_a.move(T);
%
% Notes::
% - The arrow option requires the third party package arrow3.
%
% See also TRPLOT2, TRANIMATE.



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

classdef Frame < handle

    properties
        T
        se2
        name
        hg
    end

    methods

        function f = Frame(T, varargin)

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
            opt.se2 = false;

            opt = tb_optparse(opt, varargin);

            f.se2 = opt.se2;
            f.name = opt.frame;

            % axis labels
            if isempty(opt.frame)
                fmt = '%c';
            else
                fmt = sprintf('%%c_{%s}', opt.frame);
            end

            % text label options
            if isempty(opt.text_opts)
                opt.text_opts = {};
            end
            
            if isempty(opt.axis)
                % determine some default axis dimensions
                
                d = 1.2;
                if opt.se2
                    c = transl(T);
                    d = 1.2;
                    opt.axis = [c(1)-d c(1)+d c(2)-d c(2)+d];
                else
                    % get the origin of the frame
                    if isrot(T)
                        c = [0 0 0];  % at zero for a rotation matrix
                    else
                        c = transl(T);    
                    end
                    opt.axis = [c(1)-d c(1)+d c(2)-d c(2)+d c(3)-d c(3)+d];
                end
            end
            
            % create the axes
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

            % create the transfor for the frame, this allows the whole
            % graphical structure to be easily moved
            hg = hgtransform('Parent', hax);
            f.hg = hg;
            set(f.hg, 'Tag', 'Frame');
            set(f.hg, 'UserData', f);

            if opt.se2
                % create unit vectors
                o =  [0 0]';
                x1 = [1 0]';
                y1 = [0 1]';
                
                % draw the axes
                mstart = [o o]';
                mend = [x1 y1]';
                
                if opt.arrow
                    % draw the 2 arrows
                    S = [opt.color num2str(opt.width)];
                    ha = arrow3(mstart, mend, S);
                    for h=ha'
                        set(h, 'Parent', hg);
                    end
                else
                    for i=1:2
                        plot2([mstart(i,1:2); mend(i,1:2)], ...
                            'Color', opt.color, 'Parent', hg);
                    end
                end

                
                % add the labels to each axis
                h = text(x1(1), x1(2), sprintf(fmt, 'X'), 'Parent', hg);
                if ~isempty(opt.text_opts)
                    set(h, opt.text_opts{:});
                end

                h = text(y1(1), y1(2), sprintf(fmt, 'Y'), 'Parent', hg);
                if ~isempty(opt.text_opts)
                    set(h, opt.text_opts{:});
                end

                % label the frame
                if ~isempty(opt.frame)
                    h = text(o(1)-0.04*x1(1), o(2)-0.04*y1(2), ...
                        ['\{' opt.frame '\}'], 'Parent', hg);
                    set(h, 'VerticalAlignment', 'middle', ...
                        'HorizontalAlignment', 'center', opt.text_opts{:});
                end
            else
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
                        h = plot2([mstart(i,1:3); mend(i,1:3)], ...
                            'Color', opt.color, 'Parent', hg);
                    end
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
            f.move(T);
        end

        function move(f, T)
            if f.se2
                if ~all(size(T) == [3 3])
                    error('expecting SE(2) matrix');
                end
                T = [T(1:2,1:2) zeros(2,1) T(1:2,3); 0 0 1 0; 0 0 0 1];
            elseif isrot(T)
                T = r2t(T);
            elseif ~ishomog(T)
                error('expecting SO(3) or SE(3) matrix');
            end

            % search for this named frame in all figs
            set(f.hg, 'Matrix', T);
            f.T = T;
        end

        function animate(f, P2, varargin)
        %ANIMATE Animate a coordinate frame
        %
        % ANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose P1
        % to pose P2.  Poses P1 and P2 can be represented by:
        %   - homogeneous transformation matrices (4x4)
        %   - orthonormal rotation matrices (3x3)
        %   - Quaternion
        %
        % ANIMATE(P, OPTIONS) animates a coordinate frame moving from the identity pose
        % to the pose P represented by any of the types listed above.
        %
        % ANIMATE(PSEQ, OPTIONS) animates a trajectory, where PSEQ is any of
        %   - homogeneous transformation matrix sequence (4x4xN)
        %   - orthonormal rotation matrix sequence (3x3xN)
        %   - Quaternion vector (Nx1)
        %
        % Options::
        %  'fps', fps    Number of frames per second to display (default 10)
        %  'nsteps', n   The number of steps along the path (default 50)
        %  'axis',A      Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
        %
        % See also TRPLOT.

            opt.fps = 10;
            opt.nsteps = 50;
            opt.axis = [];

            [opt, args] = tb_optparse(opt, varargin);

            P1 = [];

            % convert quaternion and rotation matrix to hom transform
            if isa(P2, 'Quaternion')
                T2 = P2.T;   % convert quaternion to transform
                if ~isempty(args) && isa(args{1},'Quaternion')
                    P1 = T2;
                    Q2 = args{1};
                    T2 = Q2.T;
                    args = args(2:end);
                else
                    T1 = eye(4,4);
                end
            elseif isrot(P2)
                T2 = r2t(P2);
                if ~isempty(args) && isrot(args{1})
                    P1 = T2;
                    T2 = r2t(args{1});
                    args = args(2:end);
                else
                    T1 = eye(4,4);
                end
            elseif ishomog(P2)
                T2 = P2;
                if ~isempty(args) && ishomog(args{1})
                    P1 = T2;
                    T2 = args{1};
                    args = args(2:end);
                else
                    T1 = eye(4,4);
                end
            end
            
            % at this point
            %   T1 is the initial pose
            %   T2 is the final pose
            %
            %  T2 may be a sequence
                
            if size(T2,3) > 1
                % tranimate(Ts)
                % we were passed a homog sequence
                if ~isempty(P1)
                    error('only 1 input argument if sequence specified');
                end
                Ttraj = T2;
            else
                % tranimate(P1, P2)
                % create a path between them
                Ttraj = ctraj(T1, T2, opt.nsteps);
            end
            
            if isempty(opt.axis)
                % create axis limits automatically based on motion of frame origin
                t = transl(Ttraj);
                mn = min(t) - 1.5;  % min value + length of axis + some
                mx = max(t) + 1.5;  % max value + length of axis + some
                axlim = [mn; mx];
                axlim = axlim(:)';
                args = [args 'axis' axlim];
            end
            
            % animate it for all poses in the sequence
            for i=1:size(Ttraj,3)
                T = Ttraj(:,:,i);
                f.move(T);
                pause(1/opt.fps);
            end
        end

        function delete(f)
        % DELETE Delete the coordinate frame
            children = get(f.hg, 'Children');
            for child=children
                delete(child);
            end
        end

        function s = char(f)
        %Link.char String representation of parameters
        %
        % s = L.char() is a string showing link parameters in compact single line format.  
        % If L is a vector of Link objects return a string with one line per Link.
        %
        % See also Link.display.
            ts = trprint(f.T, 'rpy');
            s = sprintf(' {%s} %s', f.name, ts);
            if f.se2
                s = [s ' :: SE(2)'];
            else
                s = [s ' :: SE(3)'];
            end
        end

        function display(l)
        %Frame.display Display parameters
        %
        % F.display() display link parameters in compact single line format.  If L is a 
        % vector of Link objects display one line per element.
        %
        % Notes::
        % - this method is invoked implicitly at the command line when the result
        %   of an expression is a Link object and the command has no trailing
        %   semicolon.
        %
        % See also Link.char, Link.dyn, SerialLink.showlink.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(l) );
        end % display()

        function rescale(f)
            mn = [Inf Inf Inf];
            mx = -[Inf Inf Inf];
            for frame=findobj('Tag', 'Frame')'
                T = frame.
                
            end
        end

    end % methods
end
