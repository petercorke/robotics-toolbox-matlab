function rplot(robot, q)
    
    config = robot.homeConfiguration();
    
    for i=1:length(q)
        config(i).JointPosition = q(i);
    end
    
    robot.show(config, 'PreservePlot', true);