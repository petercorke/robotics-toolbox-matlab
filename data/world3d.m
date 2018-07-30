c_obs = zeros(100, 100, 100);

c_obs(:,:,75:80) = 1;          % top floor
c_obs(10:20,10:20,75:80) = 0;  % hole through floor
c_obs(1:80,30:35,80:95) = 1;   % wall in y direction

c_obs(:,:,25:30) = 1;          % middle floor
c_obs(80:90,10:20,25:30) = 0;  % hole through floor
c_obs(50:60,1:80,30:75) = 1;   % wall in x direction

c_obs(20:100,60:70,1:25) = 1;  % bottom wall

x0 = [80 10 90];   % up the top
xg = [90 90 10];   % down the bottom



