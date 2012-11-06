function q = gait(cycle, k, offset, flip)
    k = mod(k+offset-1, numrows(cycle)) + 1;
    q = cycle(k,:);
    if flip
        q(1) = -q(1);   % for left-side legs
    end
end
