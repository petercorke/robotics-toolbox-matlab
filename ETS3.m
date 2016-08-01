classdef ETS3
    properties
        what
        param
        qvar
    end
    
    methods
        function obj = ETS3(what, x, varargin)
            if nargin > 0
                disp(['ETS3 constructing ', what]);
            else
                disp('ETS3 constructor: no args');
            end
            obj.qvar = [];
            obj.param = 0;
            
            if nargin > 1
                if isa(x, 'ETS3')
                    obj.what = x.what;
                    obj.qvar = x.qvar;
                    obj.param = x.param;
                else
                    if ischar(x)
                        obj.qvar = str2num(x(2:end));
                    else
                        obj.param = x;
                    end
                    obj.what = what;
                    
                end
%             else
%                 obj.what = what;
            end
        end
        
        function r = fkine(ets, q, varargin)
            r = SE3;
            
            opt.deg = false;
            opt = tb_optparse(opt, varargin);
            
            if opt.deg
                opt.deg = pi/180;
            else
                opt.deg = 1;
            end
            
            for e = ets
                if ~isempty(e.qvar)
                    v = q(e.qvar);
                else
                    v = e.param;
                end
                switch e.what
                    case 'Tx'
                        r = r * SE3(v, 0, 0);
                    case 'Ty'
                        r = r * SE3(0, v, 0);
                    case 'Tz'
                        r = r * SE3(0, 0, v);
                    case 'Rx'
                        r = r * SE3.Rx(v*opt.deg);
                    case 'Ry'
                        r = r * SE3.Ry(v*opt.deg);
                    case 'Rz'
                        r = r * SE3.Rz(v*opt.deg);
                end
            end
            r = r.simplify();  % simplify it if symbolic
        end
        
        function b = isjoint(ets)
            b = ~isempty(ets.qvar);
        end
        
        function k = find(ets, j)
            [~,k] = find([ets.qvar] == j);
        end
        
        function n = njoints(ets)
            n = max([ets.qvar]);
        end
        
        function s = string(ets)
            constant = 1;
            for i = 1:length(ets)
                e = ets(i);
                if e.isjoint
                    term = sprintf('%s(q%d)', e.what, e.qvar);
                else
                    term = sprintf('%s(L%d)', e.what, constant);
                    constant = constant + 1;
                end
                if i == 1
                    s = term;
                else
                    s = [s ' ' term];
                end
            end
        end
            
%         function teach(ets)
%             T = SE2;
%                         links = [];
%                         
%             for e = ets
%                 if e.isjoint
%                     if e.qvar > 1
%                         switch e.what
%                         links = [links Link('a', T.t(1))
%                 else
%                     switch e.what
%                         case 'Tx'
%                             T = T * SE2(v, 0, 0);
%                         case 'Ty'
%                             T = T * SE2(0, v, 0);
%                         case 'Rz'
%                             T = T * SE2(0, 0, v);
%                     end
%                 end
%             end
%             for j = 1:ets.njoints
%                 k = ets.find(j+1);
%                 if ~isempty(k)
%                     seq = 
% 
%             T = SE
%             for e = ets
%                 if e.isjoint
%                     l = Link(
%         end
        
        function out = mtimes(ets1, ets2)
            out = [ets1 ets2];
        end
        
        function s = structure(ets)
            s = '';
            for e = ets
                if e.qvar > 0
                    switch e.what(1)
                        case 'T'
                            s = [s 'P'];
                        case 'R'
                            s = [s 'R'];
                    end
                end
            end
        end
        
        function display(ets)
                        loose = strcmp( get(0, 'FormatSpacing'), 'loose');

            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(ets) );
        end % display()
        

        
        function s = char(ets)
            s = '';
            
            function s = render(z)
                if isa(z, 'sym')
                    s = char(z);
                else
                    s = sprintf('%g', z);
                end
            end
            
            for e = ets
                if e.isjoint
                    s = [s sprintf('%s(q%d)', e.what, e.qvar) ];
                    
                else
                    s = [s sprintf('%s(%s)', e.what, render(e.param))];
                    
                end
            end
        end
    end
    end