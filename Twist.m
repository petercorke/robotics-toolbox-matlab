classdef Twist
    properties (SetAccess = protected)
        v
        w
    end
    
    methods
        function tw = Twist(T, varargin)
            if ischar(T)
                % 'P', dir
                % 'R', dir, point 3D
                % 'R', point   2D
                switch upper(T)
                    case 'R'
                        if nargin == 2
                            % 2D case
                            
                            point = varargin{1};
                            v = -cross([0 0 1]', [point(:); 0]);
                            w = 1;
                            v = v(1:2);
                        else
                            % 3D case
                            dir = varargin{1};
                            point = varargin{2};
                            
                            w = unit(dir(:));
                            
                            v = -cross(w, point(:));
                            if nargin >= 4
                                pitch = varargin{3};
                                v = v + pitch * w;
                            end
                        end
                        
                    case {'P', 'T'}
                        dir = varargin{1};
                        
                        if length(dir) == 2
                            w = 0;
                        else
                            w = [0 0 0]';
                        end
                        v = unit(dir(:));
                end
                
                tw.v = v;
                tw.w = w;
            elseif numrows(T) == numcols(T)
                % it's a square matrix
                if T(end,end) == 1
                    % its a homogeneous matrix, take the logarithm
                    if numcols(T) == 4
                        S = trlog(T);  % use closed form for SE(3)
                    else
                        S = logm(T);
                    end
                    [skw,v] = tr2rt(S);
                    tw.v = v;
                    tw.w = vex(skw);
                else
                    % it's an augmented skew matrix, unpack it
                    [skw,v] = tr2rt(T);
                    tw.v = v;
                    tw.w = vex(skw)';
                end
            elseif numrows(T) == 1
                % its a row vector form of twist, unpack it
                switch length(T)
                    case 3
                        tw.v = T(1:2); tw.w = T(3);
                        
                    case 6
                        tw.v = T(1:3); tw.w = T(4:6);
                        
                    otherwise
                        error('RTB:Twist:badarg', '3 or 6 element vector expected');
                end
            end
        end
        
        function x = s(tw)
            x = [tw.v; tw.w]';
        end
        
        function x = S(tw)
            x = [skew(tw.w) tw.v(:)];
            x = [x; zeros(1, numcols(x))];
        end
        
        function x = expm(tw, varargin)
            if length(tw.v) == 2
                x = trexp2( tw.S, varargin{:} );
            else
                x = trexp( tw.S, varargin{:} );
            end
        end
        
        function x = T(tw, varargin)
            x = tw.expm( varargin{:} );
        end
        
        function p = pitch(tw)
            p = tw.w' * tw.v;
        end
        
        function L = line(tw)
%             L = Plucker(U = tw.w;
% V = -tw.v - tw.pitch * tw.w;
                L = Plucker('UV', tw.w, -tw.v - tw.pitch * tw.w);
        end
        
        function s = char(tw)
            s = '';
            for i=1:length(tw)
                
                ps = '( ';
                ps = [ ps, sprintf('%0.5g  ', tw.v) ];
                ps = [ ps(1:end-2), '; '];
                ps = [ ps, sprintf('%0.5g  ', tw.w) ];
                ps = [ ps(1:end-2), ' )'];
                if isempty(s)
                    s = ps;
                else
                    s = char(s, ps);
                end
            end
            

        end
        
        function display(tw)
            %Twist.display Display parameters
            %
            % L.display() displays the twist parameters in compact single line format.  If L is a
            % vector of Twist objects displays one line per element.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Twist object and the command has no trailing
            %   semicolon.
            %
            % See also Twist.char.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(tw) );
        end % display()
        
    end
end