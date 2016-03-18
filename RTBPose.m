classdef RTBPose
    
    % Base class for SO(n), SE(n)
    %
    % container for matrix data plus some methods for display and conversion
    %
    % X-X  is a matrix of elementwise differences
    % -X   is a matrix of elements with negated signs
    % dim(x) ??
    % X.isSE  true if SE2 or SE3
    % simplify(X)  map simplify over all elements
    % display(X)
    % char(X)
    % X.print  single line display
    % trprint(X)
    % [R,t] = tr2rt(X) split into rotation and translation
    % double(X) convert to matrix

    
    properties(Access=protected, Hidden=true)
        data
    end
    
    
    methods
        
        function display(obj)
            %Pose.display Display a pose
            %
            % P.display() displays the pose.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is an RTBPose subclass object and the command has no trailing
            %   semicolon.
            %
            % See also SO2, SO3, SE2, SE3.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            obj.render(inputname(1));
        end % display()
        
        function n = dim(obj)
            n = size(obj.data, 2);
        end
        
        function v = minus(a, b)
            v = a.data - b.data;
        end
        
%         function v = uminus(obj)
%             v = -v.data;
%         end
        
        % compatibility methods
        function [R,t] = tr2rt(T)
            n = numcols(T.data);

            assert(isSE(T), 'only applicable to SE2/3 class');
            if length(T) > 1
                R = zeros(3,3,length(T));
                t = zeros(length(T), 3);
                for i=1:length(T)
                    R(:,:,i) = T(i).R;
                    t(i,:) = T(i).t';
                end
            else
                R = T.R;
                t = T.t;
            end
        end
        
        function t = isSE(T)
            s = class(T);
            t = s(2) == 'E';
        end
        
        function render(obj, varname)
            
            if isa(obj(1).data, 'sym')
                disp(obj.data);
            else
                fmtR = '%10.4f';
                fmtt = '%10.4g';
                fmt0 = '%10.0f';
                if exist('cprintf')
                    print = @(color, fmt, value) cprintf(color, fmt, value);
                else
                    print = @(color, fmt, value) fprintf(fmt, value);
                end
                switch class(obj)
                    case 'SO2',  nr = 2; nt = 0;
                    case 'SE2',  nr = 2; nt = 1;
                    case 'SO3',  nr = 3; nt = 0;
                    case 'SE3',  nr = 3; nt = 1;
                end
                
                for i=1:length(obj)
                    M = obj(i).data;
                    if length(obj) > 1
                        fprintf('\n%s(%d) = \n', varname, i);
                    else
                        fprintf('\n%s = \n', varname);
                        
                    end
                    M(abs(M)<1000*eps) = 0;

                    for row=1:nr
                        for col=1:(nr+nt)
                            if col <= nr
                                % rotation matrix
                                v = M(row,col);

                                if fix(v) == v
                                    print('Errors', fmt0, v);
                                else
                                    print('Errors', fmtR, v);
                                end
                            else
                                % translation
                                print('Keywords', fmtt, M(row,col));
                            end
                        end
                        fprintf('\n');
                    end
                    % last row
                    if nt > 0
                        for col=1:(nr+nt)
                            print('Text', fmt0, M(nr+nt,col));
                        end
                        fprintf('\n');
                    end
                    
                    
                end
            end
        end
        
        function out = simplify(obj)
            out = obj;
            if isa(obj.data, 'sym')
                out.data = arrayfun(@simplify, obj.data);
            end
        end
        
        function animate(obj, varargin)
            switch class(obj)
                case 'SO2'
                    tranimate2(obj.R, varargin{:});
                    
                case 'SE2'
                    tranimate2(obj.T, varargin{:});
                    
                case 'SO3'
                    tranimate(obj.R, varargin{:});
                    
                case 'SE3'
                    tranimate(obj.T, varargin{:});
            end
        end
        
                function plot(obj, varargin)
            switch class(obj)
                case 'SO2'
                    trplot2(obj.R, varargin{:});
                    
                case 'SE2'
                    trplot2(obj.T, varargin{:});
                    
                case 'SO3'
                    trplot(obj.R, varargin{:});
                    
                case 'SE3'
                    trplot(obj.T, varargin{:});
            end
        end
        
        % conversion methods
        
        function s2 = char(obj)
            s = num2str(obj.data, '%10.4g'); %num2str(obj.data, 4);
            for i=1:numrows(s);
                s2(i,:) = ['    ', s(i,:)];
            end
        end
        
        function d = double(obj)
            d = obj.data;
        end
        
        
        function out = print(obj, varargin)
            if nargout == 0
                for T=obj
                    trprint(T.T, varargin{:});
                end
            else
                out = '';
                
                for T=obj
                    out = strvcat(out, trprint(T.T, varargin{:}));
                end
            end
        end
        
        function out = trprint(obj, varargin)
            
            if nargout == 0
                print(obj, varargin{:});
            else
                out = print(obj, varargin{:});
            end
        end
    end
    
    
end