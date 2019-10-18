function q= inverse(H, robot)
    q = robot.ikine(H,'q0',[ 1 -1 -1 3 3 1]);
end