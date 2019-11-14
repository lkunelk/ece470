function myrobot = mypuma560(DH)
    % DH - nx4, dh table each row is [theta d a alpha]
    % return SerialLink
    
    clear L;
    for i = 1:6
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L);
end

