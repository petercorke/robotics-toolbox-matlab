%plot_vehicle Plot ground vehicle pose
%
% plot_vehicle(X,OPTIONS) draw representation of ground robot as an 
% oriented triangle with pose X (1x3) [x,y,theta] or X (3x3) as homogeneous
% transform in SE(2).
%
% Options::
% 'scale',S    Draw vehicle with length S x maximum axis dimension
% 'size',S     Draw vehicle with length S
%
% See also Vehicle.plot.

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
