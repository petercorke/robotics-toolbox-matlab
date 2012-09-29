function tau = genforces(r)

    for j=1:r.n
        tau(j) = sym( sprintf('tau%d', j), 'real' );
    end
