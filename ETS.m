%ETS Elementary Transform Sequence class
%
% Manipulate a sequence (vector) of elementary transformations
%  - ETS.TX
%  - ETS.TY
%  - ETS.TZ
%  - ETS.RX
%  - ETS.RY
%  - ETS.RZ
%
% Methods::
%  ETS          Construct a sequence from string
%  isrot        True if rotational transform
%  istrans      True if translational transform
%  isjoint      Is ETS a function of qj
%  njoints      Maximum joint variable index
%  axis         Axis of translation or rotation
%  find         Find ETS that is a function of qj
%  subs         Substitute element of sequence
%-
%  eval         Evaluate ETS
%  jacobian     Compute Jacobian of ETS
%-
%  display      Display a sequence in human readable form
%  char         Convert sequence to a string
% 
% Example::
%         ets = ETS('Rx(q1)Tx(a1)Ry(q2)Ty(a3)Rz(q3)Rx(pi/2)')
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
        type %  ETS.TX, ETS.TY, ETS.TZ, ETS.RX, ETS.RY, ETS.RZ

        val
        joint  % joint number, if a joint, else 0
        constant
        sign
        symconstant
    end

    properties (Constant)
        TX = 0;
        TY = 1;
        TZ = 2;
        RX = 3;
        RY = 4;
        RZ = 5;
        
        names = {'Tx', 'Ty', 'Tz', 'Rx', 'Ry', 'Rz'};
    end
    % Element.java
%         // one of ETS.TX, ETS.TY ... ETS.RZ, DH_STANDARD/MODIFIED
% 	int		type;
% 
% 	// transform parameters, only one of these is set
% 	String	var;        // eg. q1, for joint var ETS.TYpes
% 	String	symconst;   // eg. L1, for lengths
% 	int		constant;   // eg. 90, for angles
% 
% 	// DH parameters, only set if ETS.TYpe is DH_STANDARD/MODIFIED
% 	int 	theta,	
%             alpha;
%     String  A,
%             D;
%     int     prismatic;
%     int     offset;
    
    methods
        function ets = ETS(s, varargin)
            %ETS.ETS Construct elementary transform element or sequence
            %
            % e = ETS() is a new ETS object.
            %
            % e = ETS(t) is a clone of the ETS object t and all properties are copied.
            %
            % e = ETS(op, v) is a new ETS object of type op and value v.  OP can be any
            % of
            % 'Rx'       rotation about the x-axis
            % 'Ry'       rotation about the y-axis
            % 'Rz'       rotation about the z-axis
            % 'Tx'       translation along the x-axis
            % 'Ty'       translation along the y-axis
            % 'Tz'       translation along the z-axis
            % 'transl'   sequence of finite translations along the x-, y- and z-directions.
            % 'rpy'      sequence of finite rotations about the x-, y- and z-directions.
            %
            % e = ETS(str) is a sequence of ETS objects, each described by a
            % subexpression in the string STR.  Each subexpression comprises an
            % operation as per the table above followed by parentheses and a value.
            % For example:
            %
            %         ets = ETS('Rx(q1)Tx(a1)Ry(q2)Ty(a3)Rz(q3)Rx(pi/2)')
            
            if nargin == 0
                ets.joint = false;
                ets.constant = [];
                return;
            end
            
            if isstr(s)
                if strfind(s, '(')
                    % string of tokens
                    tokens = regexp(s, '\s*(?<op>R.?|T.)\(\s*(?<arg>[^)]*)\s*\)\s*', 'names');
                    
                    joint = 1;
                    
                    ets = [];
                    for token = tokens
                        
                        % get the argument for this transform element
                        
                        % deal with case of symbolic arg or workspace expression
                        if token.arg(1) == 'q'
                            x = ETS(token.op, []);
                            x.joint = joint;
                            joint = joint+1;
                        else
                            x = ETS(token.op, token.arg);
                            x.joint = 0;
                        end
                        
                        ets = [ets x];  % append to the list
                    end
                else
                    % ETS('transl', [v])
                    % ETS('rpy', [v]);
                    switch s
                        case 'transl'
                            v = varargin{1};
                            
                            ets = [];
                            if v(1) ~= 0
                                ets = [ets ETS('Tx', v(1))];
                            end
                            if v(2) ~= 0
                                ets = [ets ETS('Ty', v(2))];
                            end
                            if v(3) ~= 0
                                ets = [ets ETS('Tz', v(3))];
                            end
                        case 'rpy'
                            v = varargin{1};
                            
                            ets = [];
                            if v(1) ~= 0
                                ets = [ets ETS('Rx', v(1))];
                            end
                            if v(2) ~= 0
                                ets = [ets ETS('Ry', v(2))];
                            end
                            if v(3) ~= 0
                                ets = [ets ETS('Rz', v(3))];
                            end
                            
                        case 'Rx'
                            ets = ETS(ETS.RX, varargin{1});
                        case 'Ry'
                            ets = ETS(ETS.RY, varargin{1});
                        case 'Rz'
                            ets = ETS(ETS.RZ, varargin{1});
                        case 'Tx'
                            ets = ETS(ETS.TX, varargin{1});
                        case 'Ty'
                            ets = ETS(ETS.TY, varargin{1});
                        case 'Tz'
                            ets = ETS(ETS.TZ, varargin{1});
                            
                        otherwise
                            error('RTB:trchain:badarg', 'unknown operator/option %s', s);
                    end
                end
                
            elseif isnumeric(s)
                ets = ETS();
                ets.type = s;
                ets.val = varargin{1};
                
            elseif isa(s, 'ETS')
                % clone the object
                ets.type = s.type;
                ets.joint = s.joint
                ets.val = s.val;
                ets.symconstant = s.symconstant;
                
                if nargin > 1
                    ets.type = varargin{1};
                end
                
                if nargin > 2
                    if varargin{2} < 0
                        ets = ets.negate();
                    end
                end
            end
        end
        
        function s = op(ets)
            s = ets.names{ets.type+1};
        end
        
        function s = char(ets)
            s = '';
            for x=ets
                e =  x.op;
                
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
        
        %-------------------------------------------------------------
        
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
        
        function b = istrans(ets)
            b = ([ets.type] >= ets(1).TX) & ([ets.type] <= ets(1).TZ)
            
        end
        
        function b = isrot(ets)
            b = ([ets.type] >= ets(1).RX) & ([ets.type] <= ets(1).RZ)
        end
        
        function b = axis(ets)
            b = char( mod([ets.type],3) + 'x' );
            
        end
        %-------------------------------------------------------------
        
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
                switch x.type
                    case ETS.RX
                        T = T * trotx(arg);
                    case ETS.RY
                        T = T * troty(arg);
                    case ETS.RZ
                        T = T * trotz(arg);
                    case ETS.TX
                        T = T * transl(arg, 0, 0);
                    case ETS.TY
                        T = T * transl(0, arg, 0);
                    case ETS.TZ
                        T = T * transl(0, 0, arg);
                    otherwise
                        error('RTB:ETS:badarg', 'unknown operator %s', x.op);
                end
            end
            
            %             if isa(q, 'symfun')
            %                 T = formula(T);
            %             end
        end
        
        function negate(ets)
            ets.constant = -ets.constant;
            
            s = ets.symconst;
            
            % add initial sign char if none
            if s(1) ~= '+' && s(1) ~= '-'
                s = ['+' s];
            end
            
            % go through the string and flip all sign chars
            kp = strfind(s, '+');
            kn = strfind(s, '-');
            s(kp) = '-';
            s(kn) = '+';
            
            % if inital sign is + remove it
            if s(1) == '+';
                s = s(2:end);
            end
            
            ets.symconst = s;
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
        
        
        
        function T = deriv(ets, q)
            switch ets.type
                case ETS.RX
                    T = r2t([0 0 0; 0 -sin(q) -cos(q); 0 cos(q) -sin(q)]);
                case ETS.RY
                    T = r2t([-sin(q) 0 cos(q); 0 0 0; -cos(q) 0 -sin(q)]);
                case ETS.RZ
                    T = r2t([-sin(q) -cos(q) 0; cos(q) -sin(q) 0; 0 0 0]);
                case ETS.TX
                    T = transl(1, 0, 0);
                case ETS.TY
                    T = transl(0, 1, 0);
                case ETS.TZ
                    T = transl(0, 0, 1);
            end
            T(4,4) = 0;
        end
        

% methods from Element.java
%  *	public boolean istrans()
%  *	public boolean isrot()
%  *	public int axis()
%  *	public boolean isjoint() {
%  *	public boolean factorMatch(int dhWhich, int i, int verbose) {
%  *	public void add(Element e) {
%  *	public Element(int ETS.TYpe, int constant) {
%  *	public Element(int ETS.TYpe)	// new of specified ETS.TYpe
%  *
%  *	public static String toString(Element [] e) {
%  *	public String argString() {
%  *	public String toString() {
%  *
%  * Constructors:
%  *	Element(Element e) 	// clone of argument
%  *	Element(Element e, int ETS.TYpe, int sign) // clone of argument with new ETS.TYpe
%  *	Element(Element e, int ETS.TYpe) // clone of argument with new ETS.TYpe
%  *	Element(String s)
 


function plus(ets, e)
end

function symPlus(ets, e)
end

function merge(ets, e)
end
        

        
        
        function b = swap(ets, next, dhWhich)
            
            b = false

		 % don't swap if both are joint variables

		if ets.isjoint() && next.isjoint()
            return
        end

        switch (dhWhich)
            case 'standard'
                % we want to sort terms into the order:	RZ	TX	TZ	RX
                
                if  ((ets.type == ETS.TZ) && (next.type == ETS.TX)) || ...
                        ((ets.type == ETS.TX) && (next.type == ETS.RX) && next.isjoint()) || ...  % push constant translations through rotational joints of the same ETS.TYpe
                        ((ets.type == ETS.TY) && (next.type == ETS.RY)) && next.isjoint() || ...
                        ((ets.type == ETS.TZ) && (next.type == ETS.RZ)) && next.isjoint() || ...
                        (~ets.isjoint() && (this.type == ETS.RX) && (next.type == ETS.TX)) || ...
                        (~ets.isjoint() && (this.type == ETS.RY) && (next.type == ETS.TY)) ||....
                        (~ets.isjoint() && ~next.isjoint() && (this.type == ETS.TZ) && (next.type == ETS.RZ)) || ...
                        ((ets.type == ETS.TY) && (next.type == ETS.TZ)) || ...  % move ETS.TY terms to the right
                        ((ets.type == ETS.TY) && (next.type == ETS.TX))
                    
                    fprintf(['Swap: ' char(ets) ' <-> ' char(next)] );
                    b = true;
                end
            case 'modified'
                if  ((ets.type == ETS.RX) && (next.type == ETS.TX)) || ...
                    ((ets.type == ETS.RY) && (next.type == ETS.TY)) || ...
                    ((ets.type == ETS.RZ) && (next.type == ETS.TZ)) || ...
                    ((ets.type == ETS.TZ) && (next.type == ETS.TX))
                    
                    fprintf(['Swap: ' char(ets) ' <-> ' char(next)] );
                    b = true;
                end
            otherwise
                error('bad DH ETS.TYpe');
        end
        b = false;
        end
        
        
%         	/**
% 	 * Substitute this transform for a triple of transforms
%      * that includes an ETS.RZ or ETS.TZ.
%      *
% 	 * @return	- null if no substituion required
% 	 *			- array of Elements to substitute
% 	 */
function s = substituteToZ(ets, prev)
    
    if nargin == 2
        switch ets.type
            case ETS.RY
                s(1) = ETS(RZ, 90);
                s(2) = ETS(ets, ETS.RX);
                s(3) = ETS(RZ, -90);
                
            case ETS.TY
                if prev.type == ETS.RZ
                    s(1) = ETS(RZ, 90);
                    s(2) = ETS(ets, ETS.TX);
                    s(3) = ETS(RZ, -90);
                else
                    s(1) = ETS(RX, -90);
                    s(2) = ETS(ets, ETS.TZ);
                    s(3) = ETS(RX, 90);
                end
                
            otherwise
                s = [];
        end
    else
        switch ets.type
            case ETS.RX
                s(1) = ETS(RY, 90);
                s(2) = ETS(ets, ETS.RZ);
                s(3) = ETS(RY, -90);
            case ETS.RY
                s(1) = ETS(RX, -90);
                s(2) = ETS(ets, ETS.RZ);
                s(3) = ETS(RX, 90);
            case ETS.TX
                s(1) = ETS(RY, 90);
                s(2) = ETS(ets, ETS.TZ);
                s(3) = ETS(RY, -90);
            case ETS.TY
                s(0) = ETS(RX, -90);
                s(1) = ETS(ets, ETS.TZ);
                s(2) = ETS(RX, 90);
            otherwise
                s = [];
        end
    end
end

% 	/**
% 	 * Simple rewriting rule for adjacent transform pairs.  Attempt to
% 	 * eliminate ETS.TY and ETS.RY.
% 	 * @param	previous element in list
% 	 * @return	- null if no substituion required
% 	 *			- array of Elements to subsitute
% 	 */
function s = substituteY(this, prev, next)
    
    s = [];
    
    if (prev.isjoint() || e.isjoint())
        return
    end
    
    % note that if rotation is -90 we must make the displacement -ve */
    if ((prev.type == ETS.RX) && (this.type == ETS.TY))
        % ETS.RX.TY -> ETS.TZ.RX
        s(1) = ETS(this, ETS.TZ, prev.constant);
        s(2) = ETS(prev);
    elseif ((prev.type == ETS.RX) && (this.type == ETS.TZ))
        % ETS.RX.TZ -> ETS.TY.RX
        s(1) = ETS(this, ETS.TY, -prev.constant);
        s(2) = ETS(prev);
    elseif ((prev.type == ETS.RY) && (this.type == ETS.TX))
        % ETS.RY.TX-> ETS.TZ.RY
        s(1) = ETS(this, ETS.TZ, -prev.constant);
        s(2) = ETS(prev);
    elseif ((prev.type == ETS.RY) && (this.type == ETS.TZ))
        % ETS.RY.TZ-> ETS.TX.RY
        s(1) = ETS(this, ETS.TX, prev.constant);
        s(2) = ETS(prev);
    elseif ((prev.type == ETS.TY) && (this.type == ETS.RX))
        % ETS.TY.RX -> ETS.RX.TZ
        s(1) = ETS(this);
        s(2) = ETS(prev, ETS.TZ, -this.constant);
        %%return s;
        s = [];
    elseif ((prev.type == ETS.RY) && (this.type == ETS.RX))
        % ETS.RY(Q).RX -> ETS.RX.RZ(-Q)
        s(1) = ETS(this);
        s(2) = ETS(prev, ETS.RZ, -1);
    elseif ((prev.type == ETS.RX) && (this.type == ETS.RY))
        % ETS.RX.RY -> ETS.RZ.RX
        s(1) = ETS(this, ETS.RZ);
        s(2) = ETS(prev);
    elseif ((prev.type == ETS.RZ) && (this.type == ETS.RX))
        % ETS.RZ.RX -> ETS.RX.RY
        s(1) = ETS(this);
        s(2) = ETS(prev, ETS.RY);
        s = [];
    end
end
    end
end

