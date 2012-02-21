% Test Jacobian primitives

p560

jacob0(p560, qz)
jacob0(p560, qr)
jacob0(p560, qn)

jacobn(p560, qz)
jacobn(p560, qr)
jacobn(p560, qn)

t = fkine(p560, qn)
tr2jac(t)
