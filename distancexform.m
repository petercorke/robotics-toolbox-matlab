%DISTANCEXFORM Distance transform
%
% D = DISTANCEXFORM(IM, OPTIONS) is the distance transform of the binary
% image IM. The elements of D have a value equal to the shortest distance
% from that element to a non-zero pixel in the input image IM.
%
% D = DISTANCEXFORM(OCCGRID, GOAL, OPTIONS) is the distance transform of
% the occupancy grid OCCGRID with respect to the specified goal point GOAL
% = [X,Y].  The cells of the grid have values of 0 for free space and 1 for
% obstacle. The resulting matrix D has cells whose value is the shortest
% distance to the goal from that cell, or NaN if the cell corresponds to an
% obstacle (set to 1 in OCCGRID).
%
% Options:
% 'euclidean'    Use Euclidean (L2) distance metric (default)
% 'cityblock'    Use cityblock or Manhattan (L1) distance metric
%
% 'animate'      Show the iterations of the computation
% 'delay',D      Delay of D seconds between animation frames (default 0.2s)
% 'movie',M      Save animation to a movie file or folder
%
% 'noipt'        Don't use Image Processing Toolbox, even if available
% 'novlfeat'     Don't use VLFeat, even if available
% 'nofast'       Don't use IPT, VLFeat or imorph, even if available.
%
% 'delay'
%
% Notes::
% - For the first case Image Processing Toolbox (IPT) or VLFeat will be used if
%   available, searched for in that order.  They use a 2-pass rather than
%   iterative algorithm and are much faster.
% - Options can be used to disable use of IPT or VLFeat.
% - If IPT or VLFeat are not available, or disabled, then imorph is used.
% - If IPT, VLFeat or imorph are not available a slower M-function is used.
% - If the 'animate' option is given then the MATLAB implementation is used.
% - Using imorph requires iteration and is slow.
%   - For the second case the Machine Vision Toolbox function imorph is required.
%   - imorph is a mex file and must be compiled.
% - The goal is given as [X,Y] not MATLAB [row,col] format.
%
% See also IMORPH, DXform, Animate.



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

function dx = distancexform(occgrid, varargin)
    
    opt.delay = 0.2;
    opt.ipt = true;
    opt.vlfeat = true;
    opt.fast = true;
    opt.metric = {'euclidean', 'cityblock'};
    opt.animate = false;
    opt.movie = [];
    [opt,args] = tb_optparse(opt, varargin);
    if opt.movie
        opt.animate = true;
    end
    if opt.animate
        opt.fast = false;
        opt.ipt = false;
        opt.vlfeat = false;
        clf
    end
    count = [];
    switch opt.metric
        case 'cityblock'
            ipt_metric = opt.metric;  % if we use bwdistgeodesic
            m = [ inf   1   inf
                1   0     1
                inf   1   inf  ];
        case 'euclidean'
            ipt_metric = 'quasi-euclidean';  % if we use bwdistgeodesic
            r2 = sqrt(2);
            m = [ r2   1   r2
                1   0    1
                r2   1   r2  ];
    end
    
    if ~isempty(args) && isvec(args{1}, 2)
        %% path planning interpretation
        %  distancexform(world, goal, metric, show)
        
        goal = args{1};
        occgrid = double(occgrid);
        
        % check the goal point is sane
        assert(occgrid(goal(2), goal(1)) == 0, 'RTB:distancexform:badarg', 'goal inside obstacle')
        
        if exist('imorph', 'file') && opt.fast
            if opt.verbose
                fprintf('using MVTB:imorph\n');
            end
            % setup to use imorph
            %   - set obstacles to NaN
            %   - set free space to Inf
            %   - set goal to 0
            occgrid(occgrid>0) = NaN;
            occgrid(occgrid==0) = Inf;
            occgrid(goal(2), goal(1)) = 0;
            
            count = 0;
            ninf = Inf;  % number of infinities in the map
            while 1
                
                occgrid = imorph(occgrid, m, 'plusmin');
                count = count+1;
                if opt.animate
                    cmap = [1 0 0; gray(count)];
                    colormap(cmap)
                    image(occgrid+1, 'CDataMapping', 'direct');
                    set(gca, 'Ydir', 'normal');
                    xlabel('x');
                    ylabel('y');
                    pause(opt.delay);
                end
                
                ninfnow = sum( isinf(occgrid(:)) ); % current number of Infs
                if ninfnow == ninf
                    % stop if the number of Infs left in the map had stopped reducing
                    % it may never get to zero if there are unreachable cells in the map
                    break;
                end
                ninf = ninfnow;
            end
            dx = occgrid;
            
        elseif exist('bwdistgeodesic', 'file') && opt.ipt
            if opt.verbose
                fprintf('using IPT:bwdistgeodesic\n');
            end
            % solve using IPT
            
            dx = double( bwdistgeodesic(occgrid==0, goal(1), goal(2), ipt_metric) );
            
        else
            if opt.verbose
                fprintf('using MATLAB code, faster if you install MVTB\n');
            end
            % setup to use M-function
            
            occgrid(occgrid>0) = NaN;
            nans = isnan(occgrid);
            occgrid(occgrid==0) = Inf;
            occgrid(goal(2), goal(1)) = 0;
            
            count = 0;
            ninf = Inf;  % number of infinities in the map
            anim = Animate(opt.movie);
            while 1
                
                occgrid = dxstep(occgrid, m);
                occgrid(nans) = NaN;
                
                count = count+1;
                if opt.animate
                    cmap = [1 0 0; gray(count)];
                    colormap(cmap)
                    image(occgrid+1, 'CDataMapping', 'direct');
                    set(gca, 'Ydir', 'normal');
                    xlabel('x');
                    ylabel('y');
                    if opt.animate
                        anim.add();
                    else
                        pause(opt.delay);
                    end
                end
                
                ninfnow = sum( isinf(occgrid(:)) ); % current number of Infs
                if ninfnow == ninf
                    % stop if the number of Infs left in the map had stopped reducing
                    % it may never get to zero if there are unreachable cells in the map
                    break;
                end
                ninf = ninfnow;
            end
            anim.close();
            dx = occgrid;
        end
        
        if opt.animate && ~isempty(count)
            fprintf('%d iterations, %d unreachable cells\n', count, ninf);
        end
        
    else
        %% image processing interpretation
        %   distancexform(world, [metric])
        
        if exist('imorph', 'file') && opt.fast
            if opt.verbose
                fprintf('using MVTB:imorph\n');
            end
            
            % setup to use imorph
            %   - set free space to Inf
            %   - set goal to 0
            occgrid = double(occgrid);
            occgrid(occgrid==0) = Inf;
            occgrid(isfinite(occgrid)) = 0;
            
            count = 0;
            anim = Animate(opt.movie);
            while 1
                occgrid = imorph(occgrid, m, 'plusmin');
                count = count+1;
                if opt.show
                    cmap = [1 0 0; gray(count)];
                    colormap(cmap)
                    image(occgrid+1, 'CDataMapping', 'direct');
                    set(gca, 'Ydir', 'normal');
                    xlabel('x');
                    ylabel('y');
                    if opt.animate
                        anim.add();
                    else
                        pause(opt.delay);
                    end
                end
                
                ninfnow = sum( isinf(occgrid(:)) ); % current number of Infs
                if ninfnow == 0
                    % stop if no Infs left in the image
                    break;
                end
            end
            anim.close();
            dx = occgrid;
        elseif exist('bwdist') && opt.ipt
            if opt.verbose
                fprintf('using IPT:bwdist\n');
            end
            % use IPT
            dx = bwdist(occgrid, ipt_metric);
            
        elseif exist('vl_imdisttf') && opt.vlfeat
            if opt.verbose
                fprintf('using VLFEAT:vl_imsdisttf\n');
            end
            im = double(occgrid);
            im(im==0) = inf;
            im(im==1) = 0;
            d2 = vl_imdisttf(im);
            dx = sqrt(d2);
            
        else
            if opt.verbose
                fprintf('using MATLAB code, faster if you install MVTB\n');
            end
            
            occgrid = double(occgrid);
            occgrid(occgrid==0) = Inf;
            occgrid(isfinite(occgrid)) = 0;
            
            count = 0;
            while 1
                occgrid = dxstep(occgrid, m);
                count = count+1;
                if opt.show
                    cmap = [1 0 0; gray(count)];
                    colormap(cmap)
                    image(occgrid+1, 'CDataMapping', 'direct');
                    set(gca, 'Ydir', 'normal');
                    xlabel('x');
                    ylabel('y');
                    pause(opt.show);
                end
                
                ninfnow = sum( isinf(occgrid(:)) ); % current number of Infs
                if ninfnow == 0
                    % stop if no Infs left in the image
                    break;
                end
            end
            dx = occgrid;
        end
    end
end

% MATLAB implementation of computational kernel

function out = dxstep(G, m)
    
    [h,w] = size(G);    % get size of occ grid
    % pad with inf
    G = [ones(1,w)*Inf; G; ones(1,w)*Inf];
    G = [ones(h+2,1)*Inf G ones(h+2,1)*Inf];
    w = w+2; h = h+2;
    
    for r=2:h-1
        for c=2:w-1
            W = G(r-1:r+1,c-1:c+1);   % get 3x3 window
            out(r-1,c-1) = min(min(W+m));  % add distances and find the minimum
        end
    end
end
