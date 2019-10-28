function myrobot = mykuka_search(delta)
    % DH - nx4, dh table each row is [theta d a alpha]
    % return SerialLink representing the robot
    a1 = 25; % [mm]
    a2 = 315;
    a3 = 35;
    a6 = 296.23 + delta(1);
    d1 = 400;
    d4 = 365;
    d6 = 161.44 + delta(2);

    DH = [
        [0, d1, a1, pi/2 ]
        [0,  0, a2, 0    ]
        [0,  0, a3, pi/2 ]
        [0, d4,  0,-pi/2 ]
        [0,  0,  0, pi/2 ]
        [0, d6, -a6, 0   ]];
    
    clear L;
    for i = 1:size(DH, 1)
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L);
end
