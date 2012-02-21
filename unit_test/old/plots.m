puma560
p1 = p560;
p2 = p560;
p2.name = 'puma #2'
p2.base = transl(0.5,-0.5,0);

clearfigs

disp('create single Puma and move it')

p1.plot(qz)
pause(1)
p1.plot(qn);

pause

disp('add another Puma to this figure and move both')

hold on
p2.plot(qn);
hold off
pause(1)
p1.plot(qz)
p2.plot(qz)
pause(1)
p1.plot(qn)
p2.plot(qn)

pause

disp('create a new figure and draw puma 1 in it')
figure
p1.plot(qz)
for i=1:5
    p1.plot(qn);
    pause(0.5);
    p1.plot(qz);
    pause(0.5);
end

pause

disp('add puma 2 to second figure')
figure(2)
hold on
p2.plot(qz)
hold off
for i=1:5
    p1.plot(qn);
    p2.plot(qn);
    pause(0.5);
    p1.plot(qz);
    p2.plot(qz);
    pause(0.5);
end
