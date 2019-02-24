function tauc = slcoulomb(robot, qd)

    Tc=reshape([robot.links.Tc],2, []);
    
    tauc = ((qd'>0).*Tc(1,:) + (qd'<0).*Tc(2,:))';
end