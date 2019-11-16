function Lab3()
    %test_plot()
    %test_motionplan()
    test_obstacle()
end

function test_plot()
    my_robot = init_robot();
    q = sample_traj();
    %obs = setupobstacle();
    %plotobstacle(obs);
    plot_robot(q, my_robot);
end

function test_motionplan()
    myrobot = init_robot();
    H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
    H1(1:3,4)= [50; 50; 10;]; % This assigns the desired displacement to the hom.tsf.mtx.
    q1 = inverse_puma(H1,myrobot);
    H2 = eul2tr([0 pi pi/2]);
    H2(1:3,4)=[50; 50; 50;];
    q2 = inverse_puma(H2,myrobot);
    
    q = motionplan(q1', q2', 0, 0, myrobot, 0, 0.01);
    plot_robot(q, myrobot)
end

function test_obstacle()
    obs = setupobstacle();
    myrobot = init_robot();
    plotobstacle(obs)
    xlim([-100, 100]);
    ylim([-100, 100]);
    zlim([0, 100]);
    plot_robot([0 0 0 0 0 0], myrobot)
end

function robot = init_robot()
    %initialize robot
    DH = [
        [0, 76, 0, pi/2]
        [0, -23.65, 43.23,0]
        [0, 0, 0, pi/2]
        [0, 43.18, 0, -pi/2]
        [0,   0, 0, pi/2]
        [0,  20, 0, 0]];
    
    robot = mypuma560(DH);
end

function q = sample_traj()
    % Forward Kinematics - 1 setup variables
    N = 200;
    q = zeros(N, 6);
    q(:, 1) = linspace(0,2*pi,N);
    q(:, 2) = linspace(0,pi/2,N);
    q(:, 3) = linspace(0,pi,N);
    q(:, 4) = linspace(pi/4,3*pi/2,N);
    q(:, 5) = linspace(-pi/3,pi/3,N);
    q(:, 6) = linspace(0,2*pi,N);
end

function plot_robot(q, robot)
    % Forward Kinemtaics - 2 Plot robot following sample trajectory
    H = forward(q,robot);
    m = H.transl;
    plot3(m(:,1),m(:,2),m(:,3),'r')
    
    robot.plot(q, 'floorlevel', 0, 'workspace', [-100, 100, -100, 100, 0, 200])
end