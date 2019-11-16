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
                oc = Hs(1:2, 4, i);
                dist = norm(obs.c - oc);
                dir = [(oc - obs.c)/norm(oc - obs.c); 0];
                shortest_dist = dist - obs.R;
                if shortest_dist < obs.rho0
                    Frep(:, i) = Frep(:, i) + zeta*(1/shortest_dist - 1/obs.rho0)/shortest_dist^2 * dir;
                end
            case 'sph'
                oc = Hs(1:3, 4, i);
                dist = norm(obs.c - oc);
                dir = (oc - obs.c)/norm(oc - obs.c);
                shortest_dist = dist - obs.R;
                if shortest_dist < obs.rho0
                    Frep(:, i) = Frep(:, i) + zeta*(1/shortest_dist - 1/obs.rho0)/shortest_dist^2 * dir;
                end
        end
    end

    % compute Jacobian
    J = jacobian(q, myrobot);
    
    % compute torques
    tau = zeros(6,1);
    for i = 1:6
        tau = tau + J(:,:,i)' * Frep(:,i);
    end
    tau = tau';
    tau = tau/norm(tau);
end

function dist_cylinder(q, myrobot, obs)
    % compute distances to cylinder for all joints
end