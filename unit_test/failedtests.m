function fails = failedtests(results)
    
    tr = table(results);
    
    f = string( tr(tr.Failed,:).Name );
    
    f = arrayfun( @(x) extractBefore(x, "/"), f);
    
    fails = unique(f);
end