function [H] = calc_H(q, link)
    % Given a link and the joint, find the H matrix
    alpha = link.alpha;
    A = link.a; % Documentation has A
    D = link.d; % 

    H(1,1) = cos(q);
    H(2,1) = sin(q);
    H(3,1) = 0;
    H(4,1) = 0;

    H(1,2) = -sin(q) * cos(alpha);
    H(2,2) = cos(q) * cos(alpha);
    H(3,2) = sin(alpha);
    H(4,2) = 0;

    H(1,3) = sin(q) * sin(alpha);
    H(2,3) = -cos(q) * sin(alpha);
    H(3,3) = cos(alpha);
    H(4,3) = 0;

    H(1,4) = A * cos(q);
    H(2,4) = A * sin(q);
    H(3,4) = D;
    H(4,4) = 1;