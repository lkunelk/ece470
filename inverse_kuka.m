function q = inverse_kuka(H, kuka)
    q = zeros(6, 1);
    
    links = kuka.links;
    a1 = links(1).a;
    a2 = links(2).a;
    a3 = links(3).a;
    a6 = links(6).a;
    d1 = links(1).d;
    d4 = links(4).d;
    d6 = links(6).d;
    
    % calculate o_c, wrist position
    R_d = H(1:3,1:3);
    o_c = H(1:3,4) - H(1:3,1:3) * [a6;0;d6];
    
    xc = o_c(1,1);
    yc = o_c(2,1);
    zc = o_c(3,1);
    
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
    q1 = q(1); 
    q2 = q(2);
    q3 = q(3);
    R_30 = [
        cos(q1)*cos(q2+q3)  sin(q1) cos(q1)*sin(q2+q3);
        sin(q1)*cos(q2+q3) -cos(q1) sin(q1)*sin(q2+q3);
        sin(q2+q3) 0 -cos(q2+q3)
    ];
    R = R_30' * R_d;
    
    q(4) = atan2(R(2,3), R(1,3));
    q(5) = atan2(sqrt(1 - R(3,3)^2), R(3,3));
    q(6) = atan2(R(3,2), -R(3,1));
end

%%
% function q = inverse_kuka(H, robot)
%     % H - 4x4 homogenous matrix representing desired position
%     % robot - SerialLink representing robot
%     
%     % q0 specifies initial guess, we chose the home configuration as
%     % initial guess
%     q = robot.ikine(H, 'q0' ,[ 0 pi/2 0 0 pi/2 0]);
% end