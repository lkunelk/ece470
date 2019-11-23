function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % q0: column vector of initial configuration angles
    % q2: column vector of final joint angles
    % t1, t2: start and end time of piecewise cubic polynomial (PWCP)
    % myrobot: robot structure
    % obs: obstacle structure
    % tol: tolerance for terminating search
    
    if isequal(size(q0), [6,1]) == 0 || isequal(size(q2), [6,1]) == 0
        error('Not a column vector!')
    end
    
    clear q % (Nx6) matrix of angles
    alpha_att = 0.01; % step size
    alpha_rep = 0.01
    q(1, :) = q0';
    while norm(q(end,1:5)-q2(1:5)') > tol
        %err = norm(q(end,1:5)-q2(1:5)')
        % get torques from attractive and repulsive forces
        tau_att = att(q(end, :)', q2, myrobot);
        tau_rep = 0;
        for j = 1:length(obs)
            tau_rep = tau_rep + rep(q(end, :)', myrobot, obs{j});
        end
        
        % update angles
        q(end+1, :) = q(end, :) + alpha_att * tau_att + alpha_rep * tau_rep;
    end
    
    % create PWCP based on q
    [h, w] = size(q);
    q(:, 6) = linspace(q0(6), q2(6), h);
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end