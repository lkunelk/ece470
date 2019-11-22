function tau = rep(q, myrobot, obs)
    % q: column vector of current joint angles
    % myrobot: the robot
    % obs: obstacles as defined in setupobstacles.m
    % return torques for each joint 
    
    zeta = 1; % tunable parameter
    Hs = forward_puma_lab3(q, myrobot);
    
    % compute F repulsive for obstacle on each joint
    Frep = zeros(3,6);
    for i = 1:6
        switch obs.type
            case 'cyl'
                % TODO: Arnav implement cylinder
                % 3 cases:
                %   - joint directly above cylinder
                %   - joint directly to the side of cylinder (same as lab3)
                %   - joint diagonally away from cylinder
                oc = Hs(1:2, 4, i);
                dist = norm(obs.c - oc);
                dir = [(oc - obs.c)/norm(oc - obs.c); 0];
                shortest_dist = dist - obs.R;
                if shortest_dist < obs.rho0
                    Frep(:, i) = Frep(:, i) + zeta*(1/shortest_dist - 1/obs.rho0)/shortest_dist^2 * dir
                end
            
            case 'workspace'
                % TODO: Nam implement force from workspace
        end
    end
    
    % compute Jacobian
    J = jacobian(q, myrobot);
    
    % compute torques
    tau = zeros(6,1);
    for i = 2:6
        tau = tau + J(:,:,i)' * Frep(:,i);
    end
    tau = tau';
    if (norm(tau) ~= 0)
        tau = tau / norm(tau);
    end
end