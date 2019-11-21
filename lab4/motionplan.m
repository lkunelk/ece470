function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % q0: column vector of initial configuration angles
    % q2: column vector of final joint angles
    % t1, t2: start and end time of piecewise cubic polynomial (PWCP)
    % myrobot: robot structure
    % obs: obstacle structure
    % tol: tolerance for terminating search
    size(q0)
    size(q2)
    q0
    q2
    if isequal(size(q0), [6,1]) == 0 || isequal(size(q2), [6,1]) == 0
        error('Not a column vector!')
    end
    
    clear q % (Nx6) matrix of angles
    alpha = .01; % step size
    q(1, :) = q0';
%     for i=1:1000
     while norm(q(end,1:5)-q2(1:5)') > tol
        err = norm(q(end,1:5)-q2(1:5)')
        % get torques from attractive and repulsive forces
        tau = att(q(end, :)', q2, myrobot)
        for j = 1:size(obs)
            tau = tau + rep(q(end, :)', myrobot, obs{j});
        end
        
        % update angles
        q(end+1, :) = q(end, :) + alpha * tau;
    end
    
    % create PWCP based on q
    % do it after testing
    [h, w] = size(q);
    q(:, 6) = linspace(q0(6), q2(6), h);
    qref = q;
end