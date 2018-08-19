function results = runall(varargin)
    
    opt.live = false;
    
    files = dir('*.m');
    
    results = table('Size', [0 2], 'VariableTypes', {'string', 'logical'}, 'VariableNames', {'File', 'Result'});
    
    for i=1:length(files)
        file = files(i).name;
        [~,mfile,~] = fileparts(file);
        
        if strcmp(mfile, 'runall')
            continue;
        end
        
        fprintf('------------------------------- %s\n', mfile)

        try
            eval(mfile)
            results = [results; {mfile, true}];
        catch
            results = [results; {mfile, false}];
        end
        
        close all
        bdclose
    end
end
    
    