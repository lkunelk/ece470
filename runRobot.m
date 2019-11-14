function runRobot = main()
    initial = 0
    final = 2*pi
    N = 200
    
    theta1 = linspace(0,2*pi,N)
    theta2 = linspace(0,pi/2,N)
    theta3 = linspace(0,pi,N)
    theta4 = linspace(pi/4,3*pi/2,N)
    theta5 = linspace(-pi/3,pi/3,N)
    theta6 = linspace(0,2*pi,N)
    q = [theta1;theta2;theta3;theta4;theta5;theta6].'
    DH = [[0, 76, 0, pi/2];[0,-23.65,43.23,0];[0,0,0,pi/2];[0,43.18,0,-pi/2];[0,0,0,pi/2];[0,20,0,0]]
    runRobot = mypuma560(DH)
    plot(runRobot,q)
end