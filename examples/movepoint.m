xg = [5 5]; % goal position
clf
axis([0 10 0 10]);
hold on
xyzlabel
grid on
xc = 5; yc = 5;
N = 8;
radius = 3;

for i=1:N
    th = (i-1)*2*pi/N;
    x0 = [xc+radius*cos(th) yc+radius*sin(th) th+pi/2];

    plot_vehicle(x0, 'r');
    r = sim('sl_drivepoint');
    y = r.find('yout');
    plot(y(:,1), y(:,2));
end
plot(xg(1), xg(2), '*')
