%TRANIMATE Animate a coordinate frame
%
% TRANIMATE(P1, P2, OPTIONS) animates a 3D coordinate frame moving from pose X1
% to pose X2.  Poses X1 and X2 can be represented by:
%   - homogeneous transformation matrices (4x4)
%   - orthonormal rotation matrices (3x3)
%   - Quaternion
%
% TRANIMATE(X, OPTIONS) animates a coordinate frame moving from the identity pose
% to the pose X represented by any of the types listed above.
%
% TRANIMATE(XSEQ, OPTIONS) animates a trajectory, where XSEQ is any of
%   - homogeneous transformation matrix sequence (4x4xN)
%   - orthonormal rotation matrix sequence (3x3xN)
%   - Quaternion vector (Nx1)
%
% Options::
%  'fps', fps    Number of frames per second to display (default 10)
%  'nsteps', n   The number of steps along the path (default 50)
%  'axis',A      Axis bounds [xmin, xmax, ymin, ymax, zmin, zmax]
%  'movie',M     Save frames as files in the folder M
%
%  Additional options are passed through to TRPLOT.
%
% Notes::
% - Uses the Animate helper class to record the frames.
% - Poses X1 and X2 must both be of the same type
% - The 'movie' options saves frames as files NNNN.png.
% - To convert frames to a movie use a command like:
%        ffmpeg -r 10 -i %04d.png out.avi
%
% See also TRPLOT, Animate.


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
%  auto detect the axis scaling
function tranimate(P2, varargin)

    opt.fps = 10;
    opt.nsteps = 50;
    opt.axis = [];
    opt.movie = [];

    [opt, args] = tb_optparse(opt, varargin);
    
    if ~isempty(opt.movie)
        anim = Animate(opt.movie);
    end
    P1 = [];

    % convert quaternion and rotation matrix to hom transform
    if isa(P2, 'Quaternion')
        T2 = P2.T;   % convert quaternion to transform
        if ~isempty(args) && isa(args{1},'Quaternion')
            T1 = T2;
            Q2 = args{1};
            T2 = Q2.T;
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif isrot(P2)
        T2 = r2t(P2);
        if ~isempty(args) && isrot(args{1})
            T1 = T2;
            T2 = r2t(args{1});
            args = args(2:end);
        else
            T1 = eye(4,4);
        end
    elseif ishomog(P2)
        T2 = P2;
        if ~isempty(args) && ishomog(args{1})
            T1 = T2;
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
    
    hg = trplot(eye(4,4), args{:});  % create a frame at the origin

    % animate it for all poses in the sequence
    for i=1:size(Ttraj,3)
        T = Ttraj(:,:,i);
        trplot(hg, T);
        
        if ~isempty(opt.movie)
            anim.add();
        end
        
        pause(1/opt.fps);
    end
