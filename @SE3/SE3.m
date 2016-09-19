%SE3 SE(3) homogeneous transformation class
%
% Inherits from:SE3 -> SO3 -> RTBPose
%
% T = SE2(X, Y, THETA) is an SE(2) homogeneous transformation (3x3)
% representing translation X and Y, and rotation THETA in the plane.
%
% T = SE2(XY) as above where XY=[X,Y] and rotation is zero
%
% T = SE2(XY, THETA) as above where XY=[X,Y]
%
% T = SE2(XYT) as above where XYT=[X,Y,THETA]
%
% Methods::
%
% Static methods::
%  SE3.Rx
%  SE3.Ry
%  SE3.Rz
%  SE3.isa(T)   Test if T is 4x4
%
% Properties::
% For single SE3 objects only, for a vector of SE3 objects use the
% equivalent methods
% t       translation as a 3x1 vector (read/write)
% R       rotation as a 3x3 matrix (read/write)
%
% Methods::
% tv      return translations as a 3xN vector
%
% Notes::
% - The properies R, t are implemented as MATLAB dependent properties.
%   When applied to a vector of SE3 object the result is a comma-separated
%   list which can be converted to a matrix by enclosing it in square
%   brackets, eg [T.t] or more conveniently using the method T.tv 
%
% See also SO3, SE2, RTBPose.

%TODO
% interp
% animate
% chain
% animate
% jacobian
% log
% norm
% print
% 
% all vectorised!
% 
%
% Superclass, provides char,display, get/set R,t,T

classdef SE3 < SO3
    
    properties (Dependent = true)
         t
%         T
R
    end
    
    methods

        function obj = SE3(varargin)
        %SE3.SE3 Create an SE(3) object
        %
        % Constructs an SE(3) pose object that contains a 4x4 homogeneous transformation matrix.
        %
        % T = SE3() is a null relative motion
        %
        % T = SE3(X, Y, Z) is an object representing pure translation defined by X,
        % Y and Z.
        %
        % T = SE3(XYZ) is an object representing pure translation defined by XYZ
        % (3x1).  If XYZ (Nx3) returns an array of SE3 objects, corresponding to
        % the rows of XYZ
        %
        % T = SE3(R, XYZ) is an object representing rotation defined by the
        % orthonormal rotation matrix R (3x3) and position given by XYZ (3x1)
        %
        % T = SE3(T) is an object representing translation and rotation defined by
        % the homogeneous transformation matrix T (3x3).  If T (3x3xN) returns an array of SE3 objects, corresponding to
        % the third index of T
        %
        % T = SE3(T) is an object representing translation and rotation defined by
        % the SE3 object T, effectively cloning the object. If T (Nx1) returns an array of SE3 objects, corresponding to
        % the index of T
        %
        % Options::
        % 'deg'         Angle is specified in degrees
        %
        % Notes::
        % - Arguments can be symbolic
        
            obj.data = eye(4,4);
            args = varargin;
            
            % if any of the arguments is symbolic the result will be symbolic
            if any( cellfun(@(x) isa(x, 'sym'), args) )
                obj.data = sym(obj.data);
            end
            
            switch length(args)
                case 0
                    % ()
                    obj.data = eye(4,4);
                    return;
                    
                case 1
                    a = args{1};
                    if isvec(a, 3)
                        % (t)
                        obj.t = a(:);
                    elseif SE3.isa(a)
                        % (SE3)
                        for i=1:size(a, 3)
                            obj(i).data = a(:,:,i);
                        end
                    elseif isa(a, 'SE3')
                        % (T)
                        for i=1:length(a)
                            obj(i).data = a(i).data;
                        end
                        
                    elseif numcols(a) == 3
                        % SE3( xyz )
                        for i=1:length(a)
                            %obj(i).data = SE3(a(i,:));
                            obj(i).data(1:3,4) = a(i,:)';
                        end
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                    
                case 2
                    a = args{1}; b = args{2};
                    
                    if (isrot(a) || SO3.isa(a)) && isvec(b,3)
                        % (R, t)
                        if isrot(a)
                            obj.data(1:3,1:3) = a;
                        else
                            obj.R(1:3,1:3) = a.R;
                        end
                        obj.data(1:3,4) = b(:);
                    else
                        error('RTB:SE3:badarg', 'unknown arguments');
                    end
                    
                case 3
                    a = args{1}; b = args{2}; c = args{3};
                    
                    obj.data(1:3,4) = [a; b; c];
                    
                otherwise
                   error('RTB:SE3:badarg', 'too many arguments');

            end
            
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  GET AND SET
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function o = get.t(obj)
            o = obj.data(1:3,4);
        end
        
        function o = set.t(obj, t)
            % TODO CHECKING
            obj.data(1:3,4) = t(:);
            o = obj;
        end
        
        function R = get.R(obj)
            R = obj.data(1:3,1:3);
        end
        
        % set.R
        
        % handle assignment SE3 = SE3, SE3=4x4, can'toverload equals

        function t = tv(obj)
            %t = zeros(3,length(obj)); FAILS FOR SYMBOLIC CASE
            for i=1:length(obj)
                t(:,i) = obj(i).data(1:3,4);
            end
        end
        
        function T = T(obj)
            for i=1:length(obj)
                T(:,:,i) = obj(i).data;
            end
        end

        function v = isidentity(obj)
            v = all(all(obj.T == eye(4,4)));
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  COMPOSITION
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        
        function out = mtimes(a, b)
            if isa(a, 'SE3') && isa(b, 'SE3')
                % SE3 * SE3
                out = repmat(SE3, 1, max(length(a),length(b)));
                if length(a) == length(b)
                    % do vector*vector and scalar*scalar case
                    for i=1:length(a)
                        out(i) = SE3( a(i).data * b(i).data);
                    end
                elseif length(a) == 1
                    % scalar*vector case
                    for i=1:length(b)
                        out(i) = SE3( a.data * b(i).data);
                    end
                elseif length(b) == 1
                    % vector*scalar case
                    for i=1:length(a)
                        out(i) = SE3( a(i).data * b.data);
                    end
                else
                    error('RTB:SE3:badops', 'invalid operand lengths to * operator');
                end
            elseif isa(a, 'SE3') && SE3.isa(b)
                % SE3 * (4x4), result is SE3
                
                out = repmat(SE3, 1, max(length(a),size(b,3)));
                if length(a) == size(b,3)
                    % do vector*vector and scalar*scalar case
                    for i=1:length(a)
                        out(i) = SE3(a(i).data * b(:,:,i));
                    end
                elseif length(a) == 1
                    % scalar*vector case
                    for i=1:length(b)
                        out(i) = SE3(a.data * b(:,:,i));
                    end
                elseif length(a) == 1
                    % vector*scalar case
                    for i=1:length(a)
                        out(i) = SE3(a(i).data * b(:,:));
                    end
                else
                    error('RTB:SE3:badops', 'invalid operand lengths to * operator');
                end
            elseif SE3.isa(a) && isa(b, 'SE3') 
                % (4x4)*SE3, result is SE3
                
                out = repmat(SE3, 1, max(size(a,3),length(b)));

                if size(a,3) == length(b)
                    % do vector*vector and scalar*scalar case
                    for i=1:length(b)
                        out(i) = SE3(a(:,:,i) * b(i).data);
                    end
                elseif size(a,3) == 1
                    % scalar*vector case
                    for i=1:length(a)
                        out(i) = SE3(a * b(i).data);
                    end
                elseif length(b) == 1
                    % vector*scalar case
                    for i=1:length(a)
                        out(i) = SE3(a(:,:,i) * b.data);
                    end
                else
                    error('RTB:SE3:badops', 'invalid operand lengths to * operator');
                end                
            elseif numrows(b) == 3
                % SE3 * vectors (3xN), result is 3xN
                
                out = zeros(4, max(length(a),numcols(b)));  % preallocate space
                b = [b; ones(1,numcols(b))];  % make homogeneous
                                
                if length(a) == numcols(b)
                    % do objvector*vector and objscalar*scalar case
                    for i=1:length(a)
                        out(:,i) = a(i).data * b(:,i);
                    end
                elseif length(a) == 1
                    % objscalar*vector case
                    for i=1:length(a)
                        out = a.data * b;
                    end
                elseif numcols(b) == 1
                    % objvector*scalar case
                    for i=1:length(a)
                        out(:,i) = a(i).data * b;
                    end
                else
                    error('RTB:SE3:badops', 'invalid operand lengths to * operator');
                end  
                
                out = out(1:3,:);
            else
                    error('RTB:SE3:badops', 'invalid operand types to * operator');
            end
        end
        

        function out = times(obj, a)
            out = mtimes(obj, a);
            
            if isa(out, 'SE3')
                % created an array of SE3's
                for i=1:length(out)
                    out(i).data = trnorm(out(i).data);
                end
            end
        end
        
       function out = mrdivide(obj, a)
            assert( isa(a, 'SE3'), 'right-hand argument must be SE3');
            
            if isa(a, 'SE3')
                % SE3 / SE3
                out = repmat(SE3, 1, max(length(obj),length(a)));
                if length(obj) == length(a)
                    % do objvector*objvector and objscalar*objscalar case
                    for i=1:length(obj)
                        out(i) = SE3( obj(i).data * a(i).inv.data);
                    end
                elseif length(obj) == 1
                    % objscalar*objvector case
                    for i=1:length(obj)
                        out(i) = SE3( obj.inv.data * a(i).data);
                    end
                elseif length(a) == 1
                    % objvector*objscalar case
                    for i=1:length(obj)
                        out(i) = SE3( obj(i).data * a.inv.data);
                    end
                else
                    error('RTB:SE3:badops', 'invalid operand lengths to / operator');
                end
                
            else
                error('RTB:SE3:badops', 'invalid operand types to / operator');
            end
        end
        
        
        function out = rdivide(obj, a)
            % do the division
            out = mrdivide(obj, a);
            
            % now normalize
            % created an array of SE3's
            for i=1:length(out)
                out(i).data = trnorm(out(i).data);
            end
        end
        
        
        function T = increment(obj, v)
            T = obj .* delta2tr(v);
        end

        function J = velxform(obj)
            
            R = obj.R;
            z = zeros(size(R));
            J = [R z; z R];
        end


        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  STANDARD SE(3) MATRIX OPERATIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function it = inv(obj)
            it = SE3( obj.R', -obj.R'*obj.t);
        end
        
        function Ti = interp(varargin)
            Ti = SE3( trinterp(varargin{:}) );
        end
        
        function m = log(obj)
            m = trlog(obj.T);
        end
        
        function m = logs(obj)
            m = trlog(obj.T);
            m = [m(1:3,4); vex(m(1:3,1:3))]';
        end
        
        function m = Ad(obj)
            %SE3.Ad  Adjoint matrix
            %
            % A = S.Ad() is the adjoint matrix (6x6) corresponding to the SE(3) value
            % S.
            %
            % See also Twist.ad.
            m = [ obj.R       skew(obj.t)*obj.R
                  zeros(3,3)  obj.R              ];
        end
        
        %% ad function
        %         function m = Ad(obj)
%             m = obj.R;
%         end
        
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  Rotation conversion wrappers
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % 
        function out = toeul(out, varargin)
            out = out.SO3.toeul(varargin{:});
        end
        
        function out = torpy(out, varargin)
            out = out.SO3.torpy(varargin{:});
        end
        
        function [a,b] = toangvec(out, varargin)
            if nargout == 1
                a = out.SO3.toangvec(varargin{:});
            else
                [a,b] = out.SO3.toangvec(varargin{:});
            end
        end
        
        % conversion methods
        
        function s = SO3(obj)
            for i=1:length(obj)
            s(i) = SO3();
            s(i).data= obj(i).R;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  VANILLA RTB FUNCTION COMPATIBILITY
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function t = transl(obj)
            t = obj.tv';
        end

        function [R,t] = tr2rt(obj)
            R = obj.R;
            t = obj.t;
        end
        
        function R = tr2r(obj)
            R = obj.R;
        end
        
        function T = trnorm(obj)
            for k=1:length(obj)
                T(k) = SE3( trnorm(obj(k).T) );
            end
        end
        
        function Pt = homtrans(T, P)
            Pt = T*P;
        end
        
        function rpy = tr2rpy(obj, varargin)
            rpy = tr2rpy(obj.T, varargin{:});
        end
        
        function eul = tr2eul(obj, varargin)
            eul = tr2eul(obj.T, varargin{:});
        end
            
        function v = ishomog(obj)
            v = true;
        end
        
        function v = ishomog2(obj)
            v = false;
        end
        
        function v = isvec(obj, n)
            v = false;
        end
            
    end
    
    methods (Static)

                
        % Static factory methods for constructors from standard representations             

        function obj = Rx(varargin)
            obj = SE3( SO3.Rx(varargin{:}) );
        end
        function obj = Ry(varargin)
            obj = SE3( SO3.Ry(varargin{:}) );
        end
        function obj = Rz(varargin)
            obj = SE3( SO3.Rz(varargin{:}) );
        end
        
        function obj = oa(varargin)
            obj = SE3( SO3.oa(varargin{:}) );
        end
        
        function obj = angvec(varargin)
            obj = SE3( SO3.angvec(varargin{:}) );
        end
        
        function obj = rpy(varargin)
            obj = SE3( SO3.rpy(varargin{:}) );
        end
        function obj = eul(varargin)
            obj = SE3( SO3.eul(varargin{:}) );
        end
        
        % Static factory methods for constructors from exotic representations             
        function obj = exp(s)
            %SE3.exp SE3 object from se(3)
            %
            %  SE3.exp(SIGMA) is the SE3 rigid-body motion given by the se(3) element SIGMA (4x4).
            %
            %  SE3.exp(exp(S) as above, but the se(3) value is expressed as a twist vector (6x1).
            %
            %  SE3.exp(SIGMA, THETA) as above, but the motion is given by SIGMA*THETA
            %  where SIGMA is an se(3) element (4x4) whose rotation part has a unit norm.
            %
            % Notes::
            % - wraps trexp.
            %
            % See also trexp.
            obj = SE3( trexp(s) );
        end
        
%         function m = ad(s)
%             %m = [skew(s(1:3)) skew(s(4:6));  skew(s(4:6)) zeros(3,3)];
%             m = [skew(s(4:6)) skew(s(1:3)) ;  zeros(3,3) skew(s(4:6)) ];
%         end
        
        function n = new(obj, varargin)
            n = SE3(varargin{:});
        end
        
        function T = check(tr)
            if isa(tr, 'SE3')
                T = tr;
            elseif SE3.isa(tr)
                T = SE3(tr);
            else
                error('expecting an SE3 or 4x4 matrix');
            end
        end
                
        function h = isa(tr, rtest)
        %SE3.ISA Test if a homogeneous transformation
        %
        % SE3.ISA(T) is true (1) if the argument T is of dimension 4x4 or 4x4xN, else
        % false (0).
        %
        % SE3.ISA(T, 'valid') as above, but also checks the validity of the rotation
        % sub-matrix.
        %
        % Notes::
        % - The first form is a fast, but incomplete, test for a transform in SE(3).
        %
        % See also SO3.ISA, SE2.ISA, SO2.ISA.
            d = size(tr);
            if ndims(tr) >= 2
                h =  all(d(1:2) == [4 4]);
                
                if h && nargin > 1
                    h = SO3.isa( tr(1:3,1:3) );
                end
            else
                h = false;
            end
        end
    end
end
