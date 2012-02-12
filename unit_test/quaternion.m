q = Quaternion()
q2 = Quaternion([1 0 0 0]);
if q ~= q2
    error('should match')
end

q = Quaternion(rotx(0.3))
q2 = Quaternion(trotx(0.3))
if q ~= q2
    error('should match')

q.R();
q.T();

q+q2
q*q2
% test the values
q/q2
q^3
q.inv()
q.norm()
q.unit()
q.scale(0)
q.scale(1)
test these equal qi and q
q.dot([1 2 3]);
q*0.1
q/0.1
q*[1 2 3];
q*[1 2 3]';
q.plot()

% quaternions

quaternion(0.1)
quaternion( mat([1,2,3]), 0.1 )
quaternion( rotx(0.1) )
quaternion( trotx(0.1) )
quaternion( quaternion(0.1) )
quaternion( mat([1,2,3,4]) )
quaternion( [1,2,3,4] )
quaternion(  mat([1,2,3]), 0.1 )

q1 = quaternion( rotx(0.1) );
q1.norm()
q1.unit()
q1.norm()
q1.double()
q1.r()
q1.tr()

q1 = quaternion( rotx(0.1) );
q2 = quaternion( roty(0.2) );
q1_t = q1
q2_t = q2
q1*2
2*q1
q1^1
q1^2
q1*q1
q1+q2
q1-q2
q1*q2
q2.inv()
inv(q2*q2)
q2*q2^(-1)
q1/q2
q1*v1
q1*v1'

q1
q2
qinterp(q1, q2, 0)
qinterp(q1, q2, 1)
qinterp(q1, q2, 0.5)
qinterp(q1, q2, [0, .2, .5, 1])

q1-q1_t
q2-q2_t

