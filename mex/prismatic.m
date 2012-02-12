stanford
stanfordm

% number of trials
n = 1;

% create random points in state space
q = rand(n, 6);
qd = rand(n, 6);
qdd = rand(n, 6);

jacobn1 = jacobn(stan,q);
jacobn2 = jacobn(stanm,q);
fprintf('Jacob N - Worst case error is %f\n', max(max(abs(jacobn1-jacobn2))));

jacob01 = jacob0(stan,q(1,:));
jacob02 = jacob0(stanm,q(1,:));
fprintf('Jacob 0 - Worst case error is %f\n', max(max(abs(jacob01-jacob02))));

tau1=rne(stan,q,qd,qdd);
tau2=rne(stanm,q,qd,qdd);

% print comparative results
fprintf('Torques and forces - Worst case error is %f\n', max(max(abs(tau1-tau2))));
