% test harness for jacob_dot

if exist('p560') == 0
    mdl_puma560
end

e = 1e-4;
col = 1;
dq=[0 0 0 0 0 0];
dq(col) = 1;

Jd = (p560.jacob0(qn+dq*e)-p560.jacob0(qn))/e;
%Jd(:,col)
Jd
jacob_dot(p560, qn, dq)

