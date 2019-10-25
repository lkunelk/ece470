function q = inverse_kuka(H_d, kuka)
    q = zeros(6, 1);
    
    links = kuka.links;
    a1 = links(1).a;
    a2 = links(2).a;
    a3 = links(3).a;
    %a6 = links(6).a;
    d1 = links(1).d;
    d4 = links(4).d;
    %d6 = links(6).d;
    
    % calculate o_c, wrist position
    R_d = H_d.R;
    o_c = H_d.t - H_d.R * [links(6).a 0 links(6).d]';
    
    xc = o_c(1);
    yc = o_c(2);
    zc = o_c(3);
    
    % calculate first 3 joint angles
    q(1) = atan2(yc, xc);
    
    r = hypot(xc, yc);
    e = hypot((zc - d1), (r - a1));
    b = hypot(d4, a3);
    alpha = atan2(a3, d4);
    D = (e^2 - a2^2 - b^2) / (2 * a2 * b);
    
    q(3) = atan2(D, +sqrt(1 - D^2)) - alpha;
    
    ang1 = q(3) - pi/2 + alpha;
    alpha2 = atan2(b * sin(ang1), a2 + b * cos(ang1));
    
    q(2) = atan2(zc-d1, r - a1) - alpha2;
    
    % solve for last 3 angles
    D2 = sin(q(1)) * R_d(1,3) - cos(q(1)) * R_d(2,3);
    
    q(5) = atan2(sqrt(1 - D2^2), D2);
    
    q(4) = atan2(R_d(1, 3), R_d(2, 3));
    
    q(5) = atan2(R_d(3, 3), real(1 - R_d(3, 3)));
    
    %q = inverse(H_d, kuka);
end

function q = inverse(H, robot)
    % H - 4x4 homogenous matrix representing desired position
    % robot - SerialLink representing robot
    
    % q0 specifies initial guess, we chose the home configuration as
    % initial guess
    q = robot.ikine(H, 'q0' ,[ 0 pi/2 0 0 pi/2 0]);
end