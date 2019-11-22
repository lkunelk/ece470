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
        if (obs.type == 'cyl')                
                % TODO: Arnav implement cylinder
                % 3 cases:
                %   - joint directly above cylinder
                %   - joint directly to the side of cylinder (same as lab3)
                %   - joint diagonally away from cylinder
                
                R = obs.R; % Radius of cylinder
                C = obs.c; % Centre position of cylinder
                rho = obs.rho0;
                H = obs.h;
                oc = Hs(1:2, 4, i);
                if (Hs(3,4,i) <= H)
                    dist = norm(C - oc);
                    dir = [(oc - C)/norm(oc - obs.c); 0];
                    shortest_dist = dist - obs.R;
                elseif (norm(C - oc) > R)
                     N = norm(oc - C);
                     point = C - R*(C - oc)/N;
                     dir = [(oc - point); (Hs(3,4,i) - H)];
                     shortest_dist = norm(Hs(1:3,4,i) - [point;H]);
                     dir = dir / norm(dir);
                else
                    shortest_dist = Hs(3,4,i) - H;
                    dir = [0;0;1];
                end
                if shortest_dist < rho
                        Frep(:, i) = Frep(:, i) + zeta*(1/shortest_dist - 1/rho)/shortest_dist^2 * dir;
                end                
        elseif (obs.type == 'workspace')
                H = obs.h;
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