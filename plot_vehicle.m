%plot_vehicle Draw mobile robot pose
%
% PLOT_VEHICLE(X,OPTIONS) draws an oriented triangle to represent the pose
% of a mobile robot moving in a planar world.  The pose X (1x3) = [x,y,theta].
% If X is a matrix (Nx3) then an animation of the robot motion is shown and
% animated at the specified frame rate.
%
% Animation mode::
%
% H = PLOT_VEHICLE(X,OPTIONS) as above draws the robot and returns a handle.
%
% PLOT_VEHICLE(X, 'handle', H) updates the pose X (1x3) of the previously drawn
% robot.
%
% Image mode::
%
% PLOT_VEHICLE(X, 'image', IMG) where IMG is an RGB image that is scaled
% and centered on the robot's position.  The vertical axis of the image
% becomes the x-axi in the plot, ie. it is rotated.  If you wish to specify
% the rotation then use 
%
% PLOT_VEHICLE(X, 'image', {IMG,R}) where R is the counterclockwise rotation angle in degrees.
% 
% Options::
%  'scale',S       draw vehicle with length S x maximum axis dimension (default
%                  1/60)
%  'size',S        draw vehicle with length S
%  'fillcolor',F   the color of the circle's interior, MATLAB color spec
%  'alpha',A       transparency of the filled circle: 0=transparent, 1=solid
%  'box'           draw a box shape (default is triangle)
%  'fps',F         animate at F frames per second (default 10)
%  'image',I       use an image to represent the robot pose
%  'retain'        when X (Nx3) then retain the robots, leaving a trail
%
% Notes::
% - The vehicle is drawn relative to the size of the axes, so set them
%   first using axis().
% - For backward compatibility, 'fill', is a synonym for 'fillcolor'
%
% See also Vehicle.plot, plot_poly.


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

% TODO needs to work for 3D point

function h_ = plot_vehicle(x, varargin)
    
    opt.scale = 1/60;
    opt.size = [];
    opt.shape = {'triangle', 'box'};
    opt.fps = 10;
    opt.handle = [];
    opt.image = [];
    opt.retain = false;
    
    [opt,args] = tb_optparse(opt, varargin);
        
    if ~isempty(opt.handle)
        % animation mode
        opt.handle.Matrix = SE2(x).SE3.T;
        
        pause(1/opt.fps)
        return
    end

    % compute some default dimensions based on axis scaling
    a = axis;
    d = (a(2)+a(4) - a(1)-a(3)) * opt.scale;
    
    % trajectory mode

    for i=1:numrows(x)
        if i==1 || opt.retain
            h = draw_robot(d, opt, args);
        end

        % animate the robot
        h.Matrix = SE2(x(i,:)).SE3.T;  % convert (x,y,th) to SE(3)
        
        pause(1/opt.fps)
    end
        if nargout > 0
            h_ = h;
        end

end

function h = draw_robot(d, opt, args)
    
    if ~isempty(opt.image)
        % display an image
        if iscell(opt.image)
            img = opt.image{1};
            rotation = 90+opt.image{2};
        else
            img = opt.image;
            rotation = 90;
        end
        h = hgtransform('Tag', 'image');
        h2 = hgtransform('Matrix', trotz(rotation, 'deg')*trscale(opt.scale)*transl(-numcols(img)/2, -numrows(img)/2,0), 'Parent', h );
        image(img, 'AlphaData', any(img>0,3), 'Parent', h2);
        axis equal
    else
        % display a simple polygon
        switch opt.shape
            case 'triangle'
                if ~isempty(opt.size)
                    d = opt.size;
                end
                L = d; W = 0.6*d;
                corners = [
                    L       0
                    -L      -W
                    -L       W]';
            case 'box'
                if ~isempty(opt.size)
                    switch length(opt.size)
                        case 1
                            W = opt.size/2; L1 = opt.size/2; L2 = opt.size/2;
                        case 2
                            W = opt.size(1)/2; L1 = opt.size(2)/2; L2 = opt.size(2)/2;
                        case 3
                            W = opt.size(1)/2; L1 = opt.size(2); L2 = opt.size(3);
                    end
                else
                    L1 = d; L2 = d; W = 0.6*d;
                end
                corners = [
                    -L1       W
                    0.6*L2   W
                    L2       0.5*W
                    L2      -0.5*W
                    0.6*L2  -W
                    -L1      -W ]';
        end
        h = plot_poly(corners, 'animate', args{:});
    end
end
