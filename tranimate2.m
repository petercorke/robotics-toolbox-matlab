%TRANIMATE2 Animate a coordinate frame
%
% TRANIMATE2(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose X1
% to pose X2.  Poses X1 and X2 can be represented by:
%   - homogeneous transformation matrices (4x4)
%   - orthonormal rotation matrices (3x3)
%
% TRANIMATE2(X, OPTIONS) animates a coordinate frame moving from the identity pose
% to the pose X represented by any of the types listed above.
%
% TRANIMATE2(XSEQ, OPTIONS) animates a trajectory, where XSEQ is any of
%   - homogeneous transformation matrix sequence (4x4xN)
%   - orthonormal rotation matrix sequence (3x3xN)
%
% Options::
%  'fps', fps    Number of frames per second to display (default 10)
%  'nsteps', n   The number of steps along the path (default 50)
%  'axis',A      Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
%  'movie',M     Save frames as a movie or sequence of frames
%  'cleanup'     Remove the frame at end of animation
%  'noxyz'       Don't label the axes
%  'rgb'         Color the axes in the order x=red, y=green, z=blue
%  'retain'      Retain frames, don't animate
%  Additional options are passed through to TRPLOT.
%
% Notes::
% - Uses the Animate helper class to record the frames.
%
% See also TRPLOT, Animate, SE3.animate.



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

% TODO
%  auto detect the axis scaling
function tranimate2(P2, varargin)

    opt.fps = 10;
    opt.nsteps = 50;
    opt.axis = [];
    opt.movie = [];
    opt.cleanup = false;
    opt.retain = false;
    opt.time = [];

    [opt, args] = tb_optparse(opt, varargin);
    
    ud.opt = opt;
    ud.args = args;
    
    if ~isempty(opt.movie)
        ud.anim = Animate(opt.movie);
    end
    P1 = [];
    if ~isempty(opt.time) && isempty(opt.fps)
        opt.fps = 1 /(opt.time(2) - opt.time(1));
    end


    % convert rotation matrix to hom transform
    if isrot2(P2)
        % tranimate2(R1, options)
        % tranimate2(R1, R2, options)
        T2 = r2t(P2);
        if ~isempty(args) && isrot(args{1})
            T1 = T2;
            T2 = r2t(args{1});
            args = args(2:end);
        else
            T1 = eye(3,3);
        end
    elseif ishomog2(P2)
        % tranimate(T1, options)
        % tranimate(T1, T2, options)
        T2 = P2;
        if ~isempty(args) && ishomog2(args{1})
            T1 = T2;
            T2 = args{1};
            args = args(2:end);
        else
            T1 = eye(3,3);
        end
    elseif isa(P2, 'function_handle')
        % we were passed a handle
        %
        % tranimate( @func(x), x, options)
        T2 = [];
        for x = args{1}
            T2 = cat(3, T2, P2(x));
        end
    end
    
    % at this point
    %   T1 is the initial pose
    %   T2 is the final pose
    %
    %  T2 may be a sequence
        
    if size(T2,3) > 1
        % tranimate2(Ts)
        % we were passed a homog sequence
        if ~isempty(P1)
            error('only 1 input argument if sequence specified');
        end
        Ttraj = T2;
    else
        % tranimate2(P1, P2)
        % create a path between them
        Ttraj = trinterp2(T1, T2, linspace(0, 1, opt.nsteps));
    end
    
    if isempty(opt.axis)
        % create axis limits automatically based on motion of frame origin
        t = transl2(Ttraj);
        mn = min(t) - 1.5;  % min value + length of axis + some
        mx = max(t) + 1.5;  % max value + length of axis + some
        axlim = [mn; mx];
        axlim = axlim(:)';
        args = [args 'axis' axlim];
    else
        args = [args 'axis' opt.axis];
    end
    
    if opt.retain
        hold on
        ud.hg = [];  % indicate no animation
    else
        ud.hg = trplot2(eye(3,3), args{:});  % create a frame at the origin
    end
    ud.Ttraj = Ttraj;

    if ~isempty(opt.time)
        ud.htime = uicontrol('Parent', gcf, 'Style', 'text', ...
            'HorizontalAlignment', 'left', 'Position', [50 20 100 20]);
    end
    % animate it for all poses in the sequence
    
    t = timer('ExecutionMode', 'fixedRate', ...
        'BusyMode', 'queue', ...
        'UserData', ud, ...
        'TasksToExecute', length(ud.Ttraj), ...
        'Period', 1/opt.fps/2);
    t.TimerFcn = @timer_callback;
    start(t);
    
    waitfor(t)
    delete(t)
        if opt.cleanup
            delete(hg);
        end
end

function guts(ud, i)
    if isa(ud.Ttraj, 'SO2')
        T = ud.Ttraj(i);
    else
        T = ud.Ttraj(:,:,i);
    end
    if ud.opt.retain
        trplot2(T, ud.args{:});
    else
        trplot2(T, 'handle', ud.hg);
    end
    
    if ~isempty(ud.opt.movie)
        anim.add();
    end
    
    if ~isempty(ud.opt.time)
        set(ud.htime, 'String', sprintf('time %g', ud.opt.time(i)));
    end
    drawnow
    
end

function timer_callback(timerObj, ~)
    ud = get(timerObj, 'UserData');
    if ~ishandle(ud.hg)
        % the figure has been closed
        stop(timerObj);
        delete(timerObj);
    end
    
    i = timerObj.TasksExecuted;
    
    guts(ud, i);
    
end