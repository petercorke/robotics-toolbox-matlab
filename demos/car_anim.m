
%%
dt = 0.2;
b = Bicycle('L', 2.6, 'speedmax', 30, 'steermax', 0.6, 'dt', dt, 'x0', [0 0 0])

b.add_driver( RandomPath(10) );

p = b.run(1000);

%%

clf
[car.image,~,car.alpha] = imread('car2.png');
car.rotation = 180;
car.centre = [81,110];
car.centre = [648; 173];
car.length = 4.2;

plotvol(10)

a = gca


x = [0 0 0]';
h=plot_vehicle(p, 'model', car, 'trail', 'r:')
a.XLimMode = 'manual';
a.YLimMode = 'manual';
set(gcf, 'Color', 'w')
grid on
a = gca;
xyzlabel




