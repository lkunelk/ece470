function H = forward(joint, robot)
    % joint - 1xn matrix, joint values
    % robot - SerialLink
    
    H = robot.fkine(joint);
end