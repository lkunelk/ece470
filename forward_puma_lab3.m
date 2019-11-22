function Hmatrix = forward_puma_lab3(joint, myrobot)
    % joint - 1xn matrix, joint values
    % robot - SerialLink
    link1 = myrobot.links(1);
    link2 = myrobot.links(2);
    link3 = myrobot.links(3);
    link4 = myrobot.links(4);
    link5 = myrobot.links(5);
    link6 = myrobot.links(6);
    
    % Hi with respect to frame i-1 
    H0_1 = calc_H(joint(1),link1);
    H1_2 = calc_H(joint(2),link2);
    H2_3 = calc_H(joint(3),link3);
    H3_4 = calc_H(joint(4),link4);
    H4_5 = calc_H(joint(5),link5);
    H5_6 = calc_H(joint(6),link6);
    
    % Hi with respect to frame 0 
    
    H0_2 = H0_1 * H1_2;
    H0_3 = H0_2 * H2_3;
    H0_4 = H0_3 * H3_4;
    H0_5 = H0_4 * H4_5;
    H0_6 = H0_5 * H5_6;
    
    Hmatrix(:,:,1) = H0_1;
    Hmatrix(:,:,2) = H0_2; 
    Hmatrix(:,:,3) = H0_3; 
    Hmatrix(:,:,4) = H0_4; 
    Hmatrix(:,:,5) = H0_5; 
    Hmatrix(:,:,6) = H0_6;
end