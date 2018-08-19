clf
set(gcf, 'Position', [90   410   651   545]);
goal = [4 8];
start = [7 2];
opt.metric = 'euclidean'
opt.show = 1
opt.movie = [];%'dxform2.mp4'

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

showpixels(cost, 'contrast', 6, 'fmt', '%.2g', 'cscale', [0 12], 'fontsize', 20, 'infsymbol', 'nancolor', 'nohidenan', 'nohideinf', 'infcolor')

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

count = 0;
ninf = 0;

while 1
    
    cost = imorph(cost, m, 'plusmin');
    count = count+1;
    if opt.show
        % transfer over the finite values
%         k = ~isinf(cost0);
%         cost(k) = cost0(k);
        
        showpixels(cost, 'contrast', 6, 'fmt', '%.2g', 'cscale', [0 14], 'fontsize', 20, 'nancolor', 'nohideinf', 'infsymbol', 'nohidenan', 'infcolor', 'here')
        xlabel('x', 'FontSize', 20); ylabel('y', 'FontSize', 20)
        %         cmap = [1 0 0; gray(count)];
        %         colormap(cmap)
        %         image(occgrid+1, 'CDataMapping', 'direct');
        %         set(gca, 'Ydir', 'normal');
        %         xlabel('x');
        %         ylabel('y');
        if ~isempty(opt.movie)
            anim.add();
        end
        pause(opt.show);
    end
    
    ninfnow = sum( isinf(cost(:)) ); % current number of Infs
    if ninfnow == ninf
        % stop if the number of Infs left in the map had stopped reducing
        % it may never get to zero if there are unreachable cells in the map
        break;
    end
    ninf = ninfnow;
end
count
if ~isempty(opt.movie)
    anim.close();
end