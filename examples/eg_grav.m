[Q2,Q3] = meshgrid(-pi:0.1:pi, -pi:0.1:pi);
for i=1:numcols(Q2),
	for j=1:numcols(Q3);
		g = p560.gravload([0 Q2(i,j) Q3(i,j) 0 0 0]);
		g2(i,j) = g(2);
		g3(i,j) = g(3);
	end
end
surfl(Q2, Q3, g2); surfl(Q2, Q3, g3);
