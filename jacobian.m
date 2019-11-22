function Jmatrix = jacobian(q, myrobot)
    
    Hmatrix = forward_puma_lab3(q, myrobot);

    O00 = [0;0;0];
    O01 = Hmatrix(1:3,4,1);
    O02 = Hmatrix(1:3,4,2);
    O03 = Hmatrix(1:3,4,3);
    O04 = Hmatrix(1:3,4,4);
    O05 = Hmatrix(1:3,4,5);
    O06 = Hmatrix(1:3,4,6);
    
    z00 = [0;0;1];
    z01 = Hmatrix(1:3,3,1);
    z02 = Hmatrix(1:3,3,2);
    z03 = Hmatrix(1:3,3,3);
    z04 = Hmatrix(1:3,3,4);
    z05 = Hmatrix(1:3,3,5);
    
    jv1 = cross(z00, (O06 - O00));
    jv2 = cross(z01, (O06 - O01));
    jv3 = cross(z02, (O06 - O02));
    jv4 = cross(z03, (O06 - O03));
    jv5 = cross(z04, (O06 - O04));
    jv6 = cross(z05, (O06 - O05));
    
    J06 = [jv1 jv2 jv3 jv4 jv5 jv6];
    
    jv1 = cross(z00, (O05 - O00));
    jv2 = cross(z01, (O05 - O01));
    jv3 = cross(z02, (O05 - O02));
    jv4 = cross(z03, (O05 - O03));
    jv5 = cross(z04, (O05 - O04));
    
    J05 = [jv1 jv2 jv3 jv4 jv5 zeros(3,1)];
        
    jv1 = cross(z00, (O04 - O00));
    jv2 = cross(z01, (O04 - O01));
    jv3 = cross(z02, (O04 - O02));
    jv4 = cross(z03, (O04 - O03));
    
    J04 = [jv1 jv2 jv3 jv4 zeros(3,2)];
    
    jv1 = cross(z00, (O03 - O00));
    jv2 = cross(z01, (O03 - O01));
    jv3 = cross(z02, (O03 - O02));
    
    J03 = [jv1 jv2 jv3 zeros(3,3)];
    
    jv1 = cross(z00, (O02 - O00));
    jv2 = cross(z01, (O02 - O01));
    
    J02 = [jv1 jv2 zeros(3,4)];
    
    jv1 = cross(z00, (O01 - O00));
    
    J01 = [jv1 zeros(3,5)];
    
    Jmatrix(:,:,1) = J01;
    Jmatrix(:,:,2) = J02; 
    Jmatrix(:,:,3) = J03; 
    Jmatrix(:,:,4) = J04; 
    Jmatrix(:,:,5) = J05; 
    Jmatrix(:,:,6) = J06;