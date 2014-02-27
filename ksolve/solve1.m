function sol = solve1(j, robot, left, right)
    % look for a constant element on the right
        which = NaN;
        for i=1:12
            if any(ismember(findq(left(i)), j)) && isempty(findq(right(i)))
                which = i;
                break;
            end
        end
        which
        if isnan(which)
            sol = [];
            return;
        end
        eq = left(which) - right(which);
        
        if robot.links(j).isrevolute()
            if hasCj(eq, j) && hasSj(eq, j)
                sol = solve_joint(eq, j );
            else
                warning('eq does not have both Sj and Cj');
            end
        else
            disp('prismatic case')
            q = sym( sprintf('q%d', j) );
            sol = solve( eq == 0, q);
        end
        
        
end
