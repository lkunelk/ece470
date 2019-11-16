function q = inverse_puma(H, puma)
    q = zeros(6, 1);
    
    links = puma.links;
    
    a2 = links(2).a;
    d1 = links(1).d;
    d2 = links(2).d; 
    d4 = links(4).d;
    d6 = links(6).d;

    q = zeros(6, 1);
    
    % calculate o_c, wrist position
    o_c = H(1:3,4) - H(1:3,1:3) * [0;0;d6];
    xc = o_c(1,1);
    yc = o_c(2,1);
    zc = o_c(3,1);
    
    % calculate first 3 joint angles
    
    r = real(sqrt((hypot(xc, yc))^2 - d2^2)) ;
    D = (r^2+(zc-d1)^2-a2^2-d4^2)/(2*a2*d4);

    q(1) = atan2(yc, xc) - atan2(-d2,r);
    q(3) = atan2(D, sqrt(1 - D^2));
    q(2) = atan2(zc-d1,r)-atan2(d4*sin(q(3)-pi/2),a2+d4*cos(q(3)-pi/2));
    
    % solve for last 3 angles
    q1 = q(1); 
    q2 = q(2);
    q3 = q(3);
    R3_0 = [cos(q1)*cos(q2+q3)  sin(q1) cos(q1)*sin(q2+q3);
            sin(q1)*cos(q2+q3) -cos(q1) sin(q1)*sin(q2+q3);
            sin(q2+q3)          0       -cos(q2+q3)];
    R = R3_0' * H(1:3,1:3);
    
    q(4) = atan2(R(2,3), R(1,3));
    q(5) = atan2(sqrt(1 - R(3,3)^2), R(3,3));
    q(6) = atan2(R(3,2), -R(3,1));
    q = q';
end