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
% 'show',D       Show the iterations of the computation, with a delay of D seconds
%                between frames.
% 'noipt'        Don't use Image Processing Toolbox, even if available
% 'novlfeat'     Don't use VLFeat, even if available
% 'nofast'       Don't use IPT, VLFeat or imorph, even if available.
%
% Notes::
% - For the first case Image Processing Toolbox (IPT) or VLFeat will be used if
%   available, searched for in that order.  They use a 2-pass rather than
%   iterative algorithm and are much faster.
% - Options can be used to disable use of IPT or VLFeat.
% - If IPT or VLFeat are not available, or disabled, then imorph is used.
% - If IPT, VLFeat or imorph are not available a slower M-function is used.
% - If the 'show' option is given then imorph is used.
%   - Using imorph requires iteration and is slow.
%   - For the second case the Machine Vision Toolbox function imorph is required.
%   - imorph is a mex file and must be compiled.
% - The goal is given as [X,Y] not MATLAB [row,col] format.
%
% See also IMORPH, DXform.



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

function d = distancexform(occgrid, varargin)
    
    opt.show = 0;
    opt.ipt = true;
    opt.vlfeat = true;
    opt.fast = true;
    opt.metric = {'euclidean', 'cityblock'};
    [opt,args] = tb_optparse(opt, varargin);
    if opt.show
        opt.ipt = false;
        opt.vlfeat = false;
    end
    
    if ~isempty(args) && isvec(args{1}, 2)
        %% path planning interpretation
        %  distancexform(world, goal, metric, show)
        
        goal = args{1};
        occgrid = double(occgrid);
        
        if exist('imorph', 'file') ~= 3
            error('Machine Vision Toolbox is required by this function');
        end
        
        switch opt.metric
            case 'cityblock'
                m = [ inf  1  inf
                        1  0    1
                      inf  1  inf  ];
            case 'euclidean'
                r2 = sqrt(2);
                m = [ r2  1 r2
                       1  0  1
                      r2  1 r2  ];
            otherwise
                error('unknown distance metric');
        end
        
        % check the goal point is sane
        if occgrid(goal(2), goal(1)) > 0
            error('goal inside obstacle')
        end
        
        if opt.fast && exist('imorph', 'file') == 3
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
                if ninfnow == ninf
                    % stop if the number of Infs left in the map had stopped reducing
                    % it may never get to zero if there are unreachable cells in the map
                    break;
                end
                ninf = ninfnow;
            end
        else
            % setup to use M-function
            
            occgrid(occgrid>0) = NaN;
            nans = isnan(occgrid);
            occgrid(occgrid==0) = Inf;
            occgrid(goal(2), goal(1)) = 0;
            
            count = 0;
            ninf = Inf;  % number of infinities in the map
            while 1
                
                occgrid = dxstep(occgrid, m);
                occgrid(nans) = NaN;

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
                if ninfnow == ninf
                    % stop if the number of Infs left in the map had stopped reducing
                    % it may never get to zero if there are unreachable cells in the map
                    break;
                end
                ninf = ninfnow;
            end
        end
        
        if opt.show
            fprintf('%d iterations, %d unreachable cells\n', count, ninf);
        end
        
        d = occgrid;
    else
        %% image processing interpretation
        %   distancexform(world, [metric])
        
        % use other toolboxes if they exist
        if opt.fast && exist('bwdist') && opt.ipt
            d = bwdist(occgrid, opt.metric);
            
        elseif exist('vl_imdisttf') == 3 && opt.vlfeat
            im = double(occgrid);
            im(im==0) = inf;
            im(im==1) = 0;
            d2 = vl_imdisttf(im);
            d = sqrt(d2);
            
        elseif opt.fast && exist('imorph', 'file') == 3
            
            switch opt.metric
                case 'cityblock'
                    m = ones(3,3);
                    m(2,2) = 0;
                case 'euclidean'
                    r2 = sqrt(2);
                    m = [r2 1 r2; 1 0 1; r2 1 r2];
                otherwise
                    error('unknown distance metric');
            end
            
            % setup to use imorph
            %   - set free space to Inf
            %   - set goal to 0
            occgrid = double(occgrid);
            occgrid(occgrid==0) = Inf;
            occgrid(isfinite(occgrid)) = 0;
            
            count = 0;
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
                    pause(opt.show);
                end
                
                ninfnow = sum( isinf(occgrid(:)) ); % current number of Infs
                if ninfnow == 0
                    % stop if no Infs left in the image
                    break;
                end
            end
            d = occgrid;
        else
            switch opt.metric
                case 'cityblock'
                    m = ones(3,3);
                    m(2,2) = 0;
                case 'euclidean'
                    r2 = sqrt(2);
                    m = [r2 1 r2; 1 0 1; r2 1 r2];
                otherwise
                    error('unknown distance metric');
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
            d = occgrid;
        end
    end
end

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
