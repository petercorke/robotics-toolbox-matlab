function s = isunit(v)
    s = abs(norm(v)-1) < 100*eps;