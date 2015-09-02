%plot_vehicle Draw ground vehicle pose
%
% PLOT_VEHICLE(X,OPTIONS) draws a representation of ground robot as an
% oriented triangle with pose X (1x3) = [x,y,theta].
%
% Options::
% 'scale',S    Draw vehicle with length S x maximum axis dimension (default
%              1/60)
% 'size',S     Draw vehicle with length S
%  'fill',F    the color of the circle's interior, MATLAB color spec
%  'alpha',A   transparency of the filled circle: 0=transparent, 1=solid.
%
% See also Vehicle.plot.

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

% TODO needs to work for 3D point

function plot_vehicle(x, varargin)
    
    opt.scale = 1/60;
    opt.size = [];
    opt.shape = {'triangle', 'box'};
    opt.retain = false;
    opt.fps = 10;
    
    [opt,args] = tb_optparse(opt, varargin);
        
    % compute some default dimensions based on axis scaling
    
    a = axis;
    d = (a(2)+a(4) - a(1)-a(3)) * opt.scale;
    
    
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
    
    for i=1:numrows(x)
        
        % convert vector form of pose to SE(2)
        T = SE2(x(i,:));
        
        if opt.retain
            % overlay instances of the robot
            plot_poly(T*corners, args{:});
        else
            % animate the robot
            if i == 1
                h = plot_poly(corners, 'moveable', args{:});
            end
            plot_poly(h, x(i,:));
            
            pause(1/opt.fps)
        end
    end
    if ~opt.retain
    delete(h)
    end
end
