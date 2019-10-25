function q = inverse_kuka(H, kuka)
    q = zeros(6, 1);
    t = H.t;
    xc = t(1)
    yc = t(2)
    zc = t(3);
    q(1) = atan2(yc, xc);
    q(1)
end