mdl_p560

% at zero pose
t = p560.fkine(qz)
t
q = p560.ikine560(t)
q
p560.fkine(q)
p560.ikine560(t, 'r')
p560.ikine560(t, 'rn')

p560.ikine(t)

% at nominal pose
qn
t = p560.fkine(qn)
t
%q = ikine560(t)
q
p560.fkine(q)
p560.ikine(t, [0, 0.7, 3, 0, 0.7, 0])

% along trajectory
[q,qd,qdd] = jtraj(qz, qr, 20)
fkine(q)

t1 = p560.fkine(qz)
t2 = p560.fkine(qr)
traj = ctraj(t1, t2, 5)
p560.ikine(traj)
