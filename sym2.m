%SYM2 Subclass of sym class
%
% This is ugly.  The provided sym class can only generate MATLAB functions, not
% expressions.  It can generate expressions in C and Fortran however.
%
% The only way to access this capability is direct to the MuPad engine, and since
% we can't change the sym class we use a subclass and add a matgen method

classdef sym2 < sym
    
    methods
        
        function s = sym2(varargin)
            s = s@sym(varargin{:});
        end
        
        function f = matgen(s,varargin)
            %MATGEN  MATLAB representation of a symbolic expression.
            %   MATGEN(S) is a fragment of MATLAB that evaluates symbolic expression S.
            %
            %
            %   See also SYM/PRETTY, SYM/LATEX, SYM/CCODE.
            %
            % Based on sym.fortran().
            
            %   Copyright 1993-2012 The MathWorks, Inc.
            
            t = inputname(1);

            if isempty(t), t = 'T'; end
            if builtin('numel',s) ~= 1, s = normalizesym(s); end
            r = sym(t);
            cleanup = onCleanup(@() mupadmex(['delete ' t ';'],0));
            mupadmex([t ':=' s.s ';']);

                f = sprintf(mupadmex('generate::MATLAB', t, 0));
                f(f == '"') = [];
                f = deblank(f);
        end
    end
end