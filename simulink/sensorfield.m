function sensor = sensorfield(x, y)
    xc = 60; yc = 90;
    sensor = 200./((x-xc).^2 + (y-yc).^2 + 200);
