%CHECK2 script to compare M-file and MEX-file versions of RNE

function check2(robot, n, args)
	robot = nofriction(robot, 'coulomb');

	% create random points in state space
	q = rand(n, 6);
	qd = rand(n, 6);
	qdd = rand(n, 6);

	% test M-file
	tic;
	tau = rne(robot, q, qd, qdd, args{:});
	t = toc;

	% test MEX-file
	tic;
	tau_f = frne(robot, q, qd, qdd, args{:});
	t_f = toc;

	% print comparative results
	fprintf('Speedup is %10.0f, worst case error is %f\n', ...
		t/t_f, max(max(abs(tau-tau_f))));
