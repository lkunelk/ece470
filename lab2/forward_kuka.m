function H = forward_kuka(joint, robot)
    % joint - 1xn matrix, joint values
    % robot - SerialLink

    H = robot.fkine(joint);
end