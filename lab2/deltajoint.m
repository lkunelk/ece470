function q=deltajoint(delta)

    kuka=mykuka_search(delta);

%--- Calibration Foundings   ----------------------------------------%
    %X1=[356.84  -293.57  28.71]';
    %X2=[738.32    15.61  30.60]';
    %X3=[371.61   248.00  30.50]';    
    %Q1=[-0.9461   0.9776   -0.6482   -0.9721    1.3806    0.2704];
    %Q2=[0.0400    0.5145    0.2967    0.0578    0.7606   -0.0419];
    %Q3=[0.8589    1.0057   -0.6969    0.8826    1.3710   -0.2368];
    
    X1 = [621.67; 104.47; 29.84]
    X2 = [628.84; -146.32; 28.82]
    X3 = [484.14; -4.13; 28.83]
    
    Q1 = [0.2096 0.7838 -0.2847 0.2314 1.1341 -0.1119]
    Q2 = [-0.333 0.7376 -0.1930 -0.3742 1.1095 0.1890]
    Q3 = [-0.0463 1.0081 -0.7894 -0.0466 1.4034 0.0084]

%% 
    H1=forward_kuka(Q1,kuka)
    H2=forward_kuka(Q2,kuka)
    H3=forward_kuka(Q3,kuka)
    
    q=norm(H1.t-X1)+norm(H2.t-X2)+norm(H3.t-X3);
end