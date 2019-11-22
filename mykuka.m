function myrobot = mykuka(DH)
    % DH - nx4, dh table each row is [theta d a alpha]
    % return SerialLink representing the robot
    
    clear L;
    for i = 1:size(DH, 1)
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L);
end
