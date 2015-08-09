classdef OccGrid < handle
    
    properties (SetAccess = private)
        grid
        scale
        origin
        offset
        xrange
        yrange
    end
    
    methods
        
        function og = OccGrid(map, varargin)
            opt.origin = [0 0];
            opt.scale = 1;
            opt.offset = false;
            
            opt = tb_optparse(opt, varargin);
            
            og.grid = map;
            og.scale = opt.scale;
            og.origin = opt.origin;
            og.offset = opt.offset;
            
            og.xrange(1) = og.origin(1);
            og.xrange(2) = og.origin(1) + (numcols(og.grid)-1)*og.scale;
            
            og.yrange(1) = og.origin(2);
            og.yrange(2) = og.origin(2) + (numrows(og.grid)-1)*og.scale;
        end
        
        function plot(og)
            cmap = [1 1 1; 1 0 0];  % non obstacles are white
            
            
            h = image(og.xrange, og.yrange, og.grid+1, 'CDataMapping', 'direct');
            set(gca, 'YDir', 'normal');
            colormap(cmap)
            grid
        end
        
        function varargout = subsref(og, S)
            switch S(1).type
                case '()'
                    u = round((S.subs{1}-og.origin(1))/og.scale)+1;
                    v = round((S.subs{2}-og.origin(2))/og.scale)+1;
                    
                    try
                        varargout{1} = og.grid(v,u);
                    catch except
                        if strcmp(except.identifier, 'MATLAB:badsubscript')
                            varargout{1} = NaN;
                        else
                            rethrow(except);
                        end
                    end
                    
                otherwise
                    [varargout{1:nargout}] = builtin('subsref', og, S);
            end
            
        end
        
        function inflate(og, iter)
            
            % quick and dirty inflator, ok for modest size sparse occupancy grids
            grid = og.grid;
            
            for i=1:iter
                [v,u] = find(grid);
                for i=1:numrows(u)
                    vset = v(i) + [-1 -1 -1 0 0 1 1 1];
                    uset = u(i) + [-1 0 1 -1 1 -1 0 1];
                    try
                        grid(vset,uset) = 1;
                    end
                end
            end
            og.grid = grid;
        end
        
        function s = char(og)
            s = sprintf('Occupancy grid: %dx%d cells', size(og.grid));
            s = char(s, sprintf('  origin: (%g,%g)', og.origin));
            s = char(s, sprintf('  span: x = [%g,%g], y = [%g,%g]', ...
                og.xrange, og.yrange));
        end
        
        function display(og)
            %Navigation.display Display status of navigation object
            %
            % N.display() displays the state of the navigation object in
            % human-readable form.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Navigation object and the command has no trailing
            %   semicolon.
            %
            % See also Navigation.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( og.char() );
        end % display()
    end
end