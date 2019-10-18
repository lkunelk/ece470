function myrobot = mypuma560(DH)    
    clear L; 
    for i = 1:6
        L(i) = Link(DH(i,:));
    end
    myrobot = SerialLink(L);
end

