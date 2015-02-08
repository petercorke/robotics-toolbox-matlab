function vellipse(robot, q, varargin)
    %SerialLink.vellipse Velocity ellipsoid for seriallink manipulator
    %
    % R.vellipse(Q, OPTIONS) displays the velocity ellipsoid for the 
    % robot R at pose Q.  The ellipsoid is centered at the tool tip position.
    %
    % Options::
    % '2d'       Ellipse for translational xy motion, for planar manipulator
    % 'trans'    Ellipsoid for translational motion (default)
    % 'rot'      Ellipsoid for rotational motion
    %
    %  Display options as per plot_ellipse to control ellipsoid face and edge
    % color and transparency.
    %
    % Example::
    %  To interactively update the velocity ellipsoid while using sliders
    %  to change the robot's pose:
    %          robot.teach('callback', @(r,q) r.vellipse(q))
    %
    % Notes::
    % - The ellipsoid is tagged with the name of the robot prepended to
    %   ".vellipse".
    % - Calling the function with a different pose will update the ellipsoid.
    %
    % See also SerialLink.jacob0, SerialLink.fellipse, plot_ellipse.
    
    name = [robot.name '.vellipse'];
    
    e = findobj('Tag', name);
    
    if isempty(q)
        delete(e);
        return;
    end
    
    opt.mode = {'trans', 'rot', '2d'};
    [opt,args] = tb_optparse(opt, varargin);
    
    if robot.n == 2
        opt.mode = '2d';
    end
    
    J = robot.jacob0(q);
    
    switch opt.mode
        case'2d'
            J = J(1:2,1:2);
        case 'trans'
            J = J(1:3,:);
        case 'rot'
            J = J(4:6,:);
    end
    
    N = (J*J');
    
    t = transl(robot.fkine(q));
    
    switch opt.mode
        case '2d'
            if isempty(e)
                h = plot_ellipse(N, t(1:2), 'edgecolor', 'r', 'Tag', name, args{:});
            else
                plot_ellipse(N, t(1:2), 'alter', e);
            end
        otherwise
            if isempty(e)
                h = plot_ellipse(N, t(1:3), 'edgecolor', 'k', 'fillcolor', 'r', 'alpha', 0.5, 'Tag', name, args{:});
            else
                plot_ellipse(N, t(1:3), 'alter', e);
            end
    end
end
