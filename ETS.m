%ETS Elementary Transform Sequence class
%
% Manipulate a sequence (vector) of elementary transformations
%  - Tx
%  - Ty
%  - Tz
%  - Rx
%  - Ry
%  - Rz
%
% Methods::
%  ETS          Construct a sequence from string
%  isjoint      Is ETS a function of qj
%  find         Find ETS that is a function of qj
%  eval         Evaluate ETS
%  jacobian     Compute Jacobian of ETS
%  njoints      Maximum joint variable index
%  display      Display a sequence in human readable form
%  char         Convert sequence to a string
% 
% Example::
%         ets = ETS('Rx(q1)Tx(a1)Ry(q2)Ty(a3)Rz(q3) Rx(pi/2)')
%         ets.eval([1 2 3]);
%
% Notes::
% - Still experimental
%
% See also trchain, trchain2.

% TODO:
%  - handle 2D case
%  - do DHFactor
%  - accept parameters from a passed struct rather as well as workspace

% Copyright (C) 1993-2015, by Peter I. Corke
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

classdef ETS < handle
    
    properties
        op % string Tx, Ty, Tz, Rx, Ry, Rz
        val
        joint  % joint number, if a joint, else 0
    end
    
    methods
        function ets = ETS(s)
            
            if nargin == 0
                return;
            end
            
            tokens = regexp(s, '\s*(?<op>R.?|T.)\(\s*(?<arg>[^)]*)\s*\)\s*', 'names');

            joint = 1;
            
            ets = [];
            for token = tokens
                
                x = ETS();
                % get the argument for this transform element
                
                % deal with case of symbolic arg or workspace expression
                if token.arg(1) == 'q'
                    x.joint = joint;
                    x.val = [];
                    joint = joint+1;
                else
                    x.joint = 0;
                    x.val = token.arg;
                end
                                
                switch token.op
                    case {'Rx', 'Ry', 'Rz', 'Tx', 'Ty', 'Tz'}
                        x.op = token.op;
                    otherwise
                        error('RTB:trchain:badarg', 'unknown operator %s', token.op);
                end
                ets = [ets x];  % append to the list
            end
            
%             if isa(q, 'symfun')
%                 T = formula(T);
%             end
        end
        
        function J = jacobian(ets, q)
            n = ets.njoints();
            J = zeros(6, n);
            for j=1:n
                % find derivative with respect to q_j
                k = ets.find(j);
                deriv = ets(k).deriv(q(j));
                pre = ets(1:k-1).eval(q);
                post = ets(k+1:end).eval(q);
                Td = pre*deriv*post
                T = ets.eval(q);
                t2r(T)*t2r(T)';
            end
        end
        
        function T = eval(ets, q)
            
            T = eye(4,4);
            
            for x = ets
                % get the argument for this transform element
                if x.isjoint()
                    % from the passed in vector q
                    
                    arg = q(x.joint);
                else            % or the workspace
                    
                    try
                        arg = evalin('base', x.val);
                    catch
                        error('RTB:ETS:badarg', 'variable %s does not exist', x.val);
                    end
                end
                
                % now evaluate the element and update the transform chain
                switch x.op
                    case 'Rx'
                        T = T * trotx(arg);
                    case 'Ry'
                        T = T * troty(arg);
                    case 'Rz'
                        T = T * trotz(arg);
                    case 'Tx'
                        T = T * transl(arg, 0, 0);
                    case 'Ty'
                        T = T * transl(0, arg, 0);
                    case 'Tz'
                        T = T * transl(0, 0, arg);
                    otherwise
                        error('RTB:ETS:badarg', 'unknown operator %s', x.op);
                end
            end
            
%             if isa(q, 'symfun')
%                 T = formula(T);
%             end
        end
        
        function T = deriv(ets, q)
            switch ets.op
                case 'Rx'
                    T = r2t([0 0 0; 0 -sin(q) -cos(q); 0 cos(q) -sin(q)]);
                case 'Ry'
                    T = r2t([-sin(q) 0 cos(q); 0 0 0; -cos(q) 0 -sin(q)]);
                case 'Rz'
                    T = r2t([-sin(q) -cos(q) 0; cos(q) -sin(q) 0; 0 0 0]);
                case 'Tx'
                    T = transl(1, 0, 0);
                case 'Ty'
                    T = transl(0, 1, 0);
                case 'Tz'
                    T = transl(0, 0, 1);
            end
            T(4,4) = 0;
        end
        
        function xo = subs(ets, k, x)
            xo = [ets(1:k-1) x ets(k+1:end)];
        end
        
        function k = find(ets, j)
            [~,k] = find([ets.joint] == j);
        end
        
                
        function n = njoints(ets)
            n = max([ets.joint]');
        end
        
        function b = isjoint(ets, j)
            b = [ets.joint] ~= 0;
        end
        
%         function varargout = subsref(this, index)
%             fprintf('-------------- %s\n', index(1).type);
%             switch index(1).type
%                 case '()'
%                     this_subset = this(index(1).subs{:});
%                     if length (index) == 1
%                         varargout = {this_subset};
%                     else
%                         varargout = cell(size(this_subset));
%                         [varargout{:}] = subsref(this_subset, index(2:end));
%                     end
%                 otherwise
% %                     for i=1:numel(this)
% %                         varargout{i} = builtin('subsref', this, index);
% %                     end
%                       z = builtin('subsref', this, index); % invoke the method
%                       77
% 
%                         varargout = cell(1,numel(z));
%                         varargout{1} = z(1);
%  
%             end
%         end
%         
%         function n = numel(this, varargin)
%             n= 1;
%         end
        
        function s = char(ets)
            s = '';
            for x=ets
                e = x.op;
                
                if isjoint(x)
                    e = [e '(q' num2str(x.joint) ')'];
                else
                    e = [e '(' num2str(x.val) ')'];
                end
                s = strcat(s, e);
            end
        end
        
        function display(ets)
            %ETS.display Display parameters
            %
            % ETS.display() displays the transform parameters in compact single line format.
            %
            % Notes::
            % - This method is invoked implicitly at the command line when the result
            %   of an expression is a Link object and the command has no trailing
            %   semicolon.
            %
            % See also Link.char, Link.dyn, SerialLink.showlink.
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( char(ets) );
        end % display()
    end
end
