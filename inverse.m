function q = inverse(H, robot)
    % H - 4x4 homogenous matrix representing desired position
    % robot - SerialLink representing robot
    
    % q0 specifies initial guess, we chose a random guess that works
    q = robot.ikine(H, 'q0' ,[ 1 -1 -1 3 3 1]);
end