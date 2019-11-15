function tau = att(q1, q2, myrobot)
    % 
    % q: column vector of current angles
    
    % q2: column vector of final angles
    % myrobot: robot structure
    % return tau: row vectors of torques on each joint
    
    % step1: compute forward kinematics of each origin of current angle
    % step2: compute forward kinematics of each origin of final angles
    Hq1 = forward_puma_lab3(q1,myrobot);
    Hq2 = forward_puma_lab3(q2,myrobot);
    
    c = [1 1 1 1 1 1];
    d = 10000; % Found a very high number
    % step3: compute artificial forces
    Fatt = zeros(3,6)
    for i = 1:6
       dist = norm(Hq2(1:3,4,i) - Hq1(1:3,4,i))
       if (dist < d)
           Fatt(:,i) = - c(i) * (Hq1(1:3,4,i) - Hq2(1:3,4,i))
       else
           Fatt(:,i) = - c(i) * ((Hq1(1:3,4,i) - Hq2(1:3,4,i)) / dist) * d
       end
    end
    % step4: compute Jacobians
    J = jacobian(q1, myrobot);
    
    % step5: compute torques
     tau = zeros(6,1);
     for i = 1:6
        tau = tau + J(:,:,i)' * Fatt(:,i);
     end
     
     tau = tau / norm(tau);
%     tau = tau';
end