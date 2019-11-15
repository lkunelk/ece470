function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    % q0: column vector of initial configuration angles
    % q2: column vector of final joint angles
    % t1, t2: start and end time of piecewise cubic polynomial (PWCP)
    % myrobot: robot structure
    % obs: obstacle structure
    % tol: tolerance for terminating search
    
    clear q % (Nx6) matrix of angles
    alpha = .1; % step size
    q(1, :) = q0';
    
    % while
    for i=1:10
        % get torques from attractive and repulsice forces
        %tau = att(q(end, :)', q2, myrobot)
        tau = [2 2 2 2 1 1]
        
        % update angles
        q(end+1, :) = q(end, :) + alpha * tau / norm(tau);
    end
    
    % create PWCP based on q
    % do it after testing
    
    qref = q;
end