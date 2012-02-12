% target loine
L = [1 -2 4];
clf
axis([0 10 0 10]);
hold on
xyzlabel
x = [0 10];
y = -(L(1)*x+L(3))/L(2);
plot(x, y, 'k--');
grid on

xc = 5; yc = 5;
N = 4;
radius = 3;

for i=1:N
    th = (i-1)*2*pi/N;
    x0 = [xc+radius*cos(th) yc+radius*sin(th) th+pi/2];

    plot_vehicle(x0, 'r');
    r = sim('sl_driveline');
    y = r.find('yout');
    plot(y(:,1), y(:,2));
end
