function check
    
    fprintf('***************************************************************\n')
    fprintf('************************ Puma 560 *****************************\n')
    fprintf('***************************************************************\n')
    clear
    mdl_puma560
    mdl_puma560akb
    
    if exist('rne') == 3
        error('need to remove rne.mex file from @SerialLink dir');
    end
    
    check1(p560, p560m);
    
    
    fprintf('\n***************************************************************\n')
    fprintf('********************** Stanford arm ***************************\n')
    fprintf('***************************************************************\n')
    
    clear
    mdl_stanford
    stanfordm   % non standard model shipped in this folder
    
    check1(stanf, stanm);
    
end


function check1(rdh, rmdh)
    %CHECK script to compare M-file and MEX-file versions of RNE
    
    % load the model and remove non-linear friction
    rdh = nofriction(rdh, 'coulomb');
    rmdh = nofriction(rdh, 'coulomb');
    
    % number of trials
    n = 10;
    
    fprintf('************************ normal case *****************************\n')
    args = {};
    fprintf('DH:  ')
    check2(rdh, n, args);
    fprintf('MDH: ')
    check2(rmdh, n, args);
    
    fprintf('************************ no gravity *****************************\n')
    args = {[0 0 0]};
    fprintf('DH:  ')
    check2(rdh, n, args);
    fprintf('MDH: ')
    check2(rmdh, n, args);
    
    fprintf('************************ ext force *****************************\n')
    args = {[0 0 9.81], [10 10 10 10 10 10]'};
    fprintf('DH:  ')
    check2(rdh, n, args);
    fprintf('MDH: ')
    check2(rmdh, n, args);
    
end

%CHECK2 script to compare M-file and MEX-file versions of RNE

function check2(robot, n, args)
    robot = nofriction(robot, 'coulomb');
    
    % create random points in state space
    q = rand(n, 6);
    qd = rand(n, 6);
    qdd = rand(n, 6);
    
    % test M-file
    robot.fast = 0;
    tic;
    tau = rne(robot, q, qd, qdd, args{:});
    t = toc;
    
    % test MEX-file
    robot.fast = 1;
    tic;
    tau_f = rne(robot, q, qd, qdd, args{:});
    t_f = toc;
    
    % print comparative results
    fprintf('Speedup is %10.0f, worst case error is %f\n', ...
        t/t_f, max(max(abs(tau-tau_f))));
end