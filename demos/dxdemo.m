%DXDEMO Demonstrate distance transform planner using animation
%
% MORPHDEMO(IM, SE, OPTIONS) displays an animation to show the principles
% of the mathematical morphology operations dilation or erosion.  Two
% windows are displayed side by side, input binary image on the left and
% output image on the right.  The structuring element moves over the input
% image and is colored red if the result is zero, else blue.  Pixels in
% the output image are initially all grey but change to black or white
% as the structuring element moves.
%
% OUT = MORPHDEMO(IM, SE, OPTIONS) as above but returns the output image.
%
% Options::
% 'dilate'      Perform morphological dilation
% 'erode'       Perform morphological erosion
% 'delay'       Time between animation frames (default 0.5s)
% 'scale',S     Scale factor for output image (default 64)
% 'movie',M     Write image frames to the folder M
%
% Notes::
% - This is meant for small images, say 10x10 pixels.
%
% See also IMORPH, IDILATE, IERODE.

function out = dxdemo(map, goal, varargin)
    
    opt.delay = 0;
    opt.movie = [];
    opt.scale = 64;
    opt.metric = {'euclidean', 'manhattan'};
    opt = tb_optparse(opt, varargin);
    
    
    set(gcf, 'Position', [90         435        1355         520])
    clf
    
    
    goal = [4 8];
    start = [7 2];
    opt.metric = 'euclidean';
    opt.movie = [];%'dxform2.mp4'
    opt.delay = 0;
    
    % make a simple map
    occgrid = zeros(10,10);
    occgrid(4:6,3:7) = 1;
    %occgrid(7:8,7) = 1;  % extra bit
    
    
    cost0 = occgrid;
    cost0(cost0==1) = NaN;
    
    cost = cost0;
    cost(cost0==0) = Inf;
    cost(goal(2), goal(1)) = 0;
    
    if ~isempty(opt.movie)
        anim = Animate(opt.movie);
    end
    
    if ~isempty(opt.movie)
        anim.add();
    end
    
    
    switch opt.metric
        case 'cityblock'
            m = [inf 1 inf
                1  0  1
                inf 1 inf];
        case 'euclidean'
            r2 = sqrt(2);
            m = [r2 1 r2
                1 0  1
                r2 1 r2];
        otherwise
            error('unknown distance metric');
    end
    
    iteration = 0;
    ninf = 0;
    n2 = 1; % half width
    n22 = 1.5;
    
    newcost = inf(size(cost));
    
    title('Wavefront path planning simulation');
    
    while true
        iteration = iteration+1;
        
        maxval = max(max(cost(isfinite(cost))));
        
        subplot(121)
        showpixels(cost, 'contrast', maxval*0.7, 'fmt', '%.2g', 'cscale', [0 maxval+2], 'fontsize', 20, 'nancolor', 'nohideinf', 'infsymbol', 'nohidenan', 'infcolor')
        xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20);
            hpatch1 = patch(1, 1,  'y', 'FaceAlpha', 0.5);
        
        for r=n2+1:numrows(cost)-n2
            for c=n2+1:numcols(cost)-n2
                
                win = cost(r-n2:r+n2, c-n2:c+n2);
                
                if isnan(cost(r,c))
                    newcost(r,c) = NaN;
                else
                    newcost(r,c) = min(min(win+m));
                end
                
                % animate the patch
                hpatch1.XData = [c-n22 c+n22 c+n22 c-n22];
                hpatch1.YData = [r-n22 r-n22 r+n22 r+n22];
                
                
                subplot(122)
                cla
                showpixels(newcost, 'contrast', maxval*0.7, 'fmt', '%.2g', 'cscale', [0 maxval+2], 'fontsize', 20, 'nancolor', 'nohideinf', 'infsymbol', 'nohidenan', 'infcolor')
                xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20)
                    hpatch2 = patch([c-0.5 c+0.5 c+0.5 c-0.5], [r-0.5 r-0.5 r+0.5 r+0.5],  'y', 'FaceAlpha', 0.5);

                
                if ~isempty(opt.movie)
                    anim.add();
                end
                if opt.delay == 0
                    drawnow
                else
                    pause(opt.delay);
                end
            end
        end
        
        cost = newcost;
        ninfnow = sum(sum( isinf(cost(2:end-1,2:end-1)) )) % current number of Infs
        if ninfnow == 0 || ninfnow == ninf
            % stop if the number of Infs left in the map had stopped reducing
            % it may never get to zero if there are unreachable cells in the map
            break;
        end
        ninf = ninfnow;
        

    
    if ~isempty(opt.movie)
        anim.close();
    end
end