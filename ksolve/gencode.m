function s = gencode(Q)
    
    s = 'function q = xikine(T, sol)';
    s = G(s, '  if nargin < 2; sol = ones(1, %d); end', length(Q));
    s = G(s, '  px = T(1,4); py = T(2,4); pz = T(3,4);');
    
    for j=1:3
        Qj = Q{j};   % cast it to subclass
        if length(Qj) == 1
            s = G(s, '  q(%d) = %s', j, matgen2(Qj));
        elseif length(Qj) == 2
            s = G(s, '  if sol(%d) == 1', j);
            s = G(s, '    q(%d) = %s', j, matgen2(Qj(1)));
            s = G(s, '  else');
            s = G(s, '    q(%d) = %s', j, matgen2(Qj(2)));
            s = G(s, '  end');
            
            
        end
        
        
        s = G(s, '  S%d = sin(q(%d));', j, j);
        s = G(s, '  C%d = cos(q(%d));', j, j);
        s = G(s, ' ');
        
        
    end
    s = G(s, 'end');
    
    fp = fopen('xikine.m', 'w');
    for i=1:numrows(s)
    fprintf(fp, '%s\n', deblank(s(i,:)));
    end
    fclose(fp);
    
end

function s = G(s, fmt, varargin)
    
    s = strvcat(s, sprintf(fmt, varargin{:}));
end

function s = matgen2(e)
    
    s = matgen(sym2(e));
    
    k = strfind(s, '=');
    s = deblank( s(k+2:end) );
end
