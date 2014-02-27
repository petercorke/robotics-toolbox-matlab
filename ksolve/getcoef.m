function coef = getcoef(eq, trig)
    z = children( collect(eq, trig) );
    z = children( z(1) );
    coef = z(1);
end
