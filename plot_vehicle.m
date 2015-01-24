%plot_vehicle Draw ground vehicle pose
%
% PLOT_VEHICLE(X,OPTIONS) draws a representation of ground robot as an
% oriented triangle with pose X (1x3) = [x,y,theta] or X (3x3) as an SE(2)
% homogeneous transform.
%
% Options::
% 'scale',S    Draw vehicle with length S x maximum axis dimension (default
%              1/60)
% 'size',S     Draw vehicle with length S
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
    
    [opt,args] = tb_optparse(opt, varargin);
    
    % get the current axes
    a = axis;
    
    % compute the dimensions of the robot
    if ~isempty(opt.size)
        d = opt.size;
    else
        d = (a(2)+a(4) - a(1)-a(3)) * opt.scale;
    end
    
    if numel(x) == 3
        % convert vector form of pose to SE(2)
        T = se2(x(1), x(2), x(3));
    else
        T = x;
    end
    
    % draw it
%     points = [
%         d 0 1
%         -d -0.6*d 1
%         -d 0.6*d 1
%         d 0 1]';
    
        points = [
        d 0
        -d -0.6*d
        -d 0.6*d]';

    points = homtrans(T, points);
    
    plot_poly(points, args{:});
        
    end
