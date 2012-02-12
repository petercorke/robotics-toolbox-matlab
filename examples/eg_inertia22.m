Q3 = -pi:0.1:pi;
for j=1:numcols(Q3);
    M = p560.inertia([0 0 Q3(j) 0 0 0]);
    M22(j) = M(2,2);
end
plot(Q3, M22)
xlabel('q_3 (rad)');
ylabel('M_{22}');
