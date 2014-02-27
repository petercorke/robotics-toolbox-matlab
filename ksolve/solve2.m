function sol = solve2(j, left, right)
    % look for a simple trig term on the right
    
    [bs,ss] = isSj(right, j);
    [bc,sc] = isCj(right, j);
    
    if ~(any(bs) && any(bc))
        sol = [];
        return;
    end
    
    ks = find(bs, 1);
    kc = find(bc, 1);
    
    eq_sin = ss(ks) * left(ks);
    eq_cos = sc(kc) * left(kc);
    
    sol = atan2(eq_sin, eq_cos);
    
    disp('solve2')
    sol
    
end
