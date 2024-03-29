function Lab3()
    puma = init_robot();
    
    % uncomment run 1 function at a time
    %test_plot_sample_traj(puma)
    
    % part 2
    %part2_test_motion_plan(puma)
    
    % part 1 and 3
    %part3_test_sphere_cylinder(puma)
    test_obstacle(puma)
end

function test_plot_sample_traj(myrobot)
    q = sample_traj();
    plot_robot(q, myrobot);
end

function part2_test_motion_plan(myrobot)
    % only test with attractive forces
    H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
    H1(1:3,4)= [50; 50; 10;]; % This assigns the desired displacement to the hom.tsf.mtx.
    q1 = inverse_puma(H1, myrobot);
    
    H2 = eul2tr([0 pi pi/2]);
    H2(1:3,4)=[20; -20; 120;];
    q2 = inverse_puma(H2,myrobot);
    
    qref = motionplan(q1', q2', 0, 10, myrobot, [], 0.02);
    t = linspace(0,10,300);
    q = ppval(qref,t)';
    plot_robot(q, myrobot)
end

function part3_test_sphere_cylinder(myrobot)
    H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx>H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
    H1(1:3,4)=100*[-1; 3; 3;]/4;
    q1 = inverse_puma(H1,myrobot);
    
    H2 = eul2tr([0 pi -pi/2]);
    H2(1:3,4)=100*[3; -1; 2;]/4;
    q2 = inverse_puma(H2,myrobot);
    
    tau = att(q1,q2,myrobot);
    
    obs = setupobstacle();
    q3 = 0.9*q1+0.1*q2;
    tau = rep(q3, myrobot, obs{1}) % This tests the torque for the cylinder obstacle
    expected = [0.9950 0.0291 -0.0504 0.0790 0.0197 0.0000]
    
    q = [pi/2 pi 1.2*pi 0 0 0];
    tau = rep(q,myrobot,obs{6})
    expected = [-0.1138 -0.2140 -0.9702 0 -0.0037 0]
    
    plotobstacle(obs)
    xlim([-100, 100]);
    ylim([-100, 100]);
    zlim([0, 200]);
    plot_robot(q3, myrobot)
end

function test_obstacle(myrobot)
    H1 = eul2tr([0 pi pi/2]); % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx>H1(1:3,4)=100*[-1; 3; 3;]/4; % This assigns the desired displacement to the hom.tsf.mtx.
    H1(1:3,4)=100*[-1; 3; 3;]/4;
    q1 = inverse_puma(H1,myrobot);
    
    H2 = eul2tr([0 pi -pi/2]);
    H2(1:3,4)=100*[3; -1; 2;]/4;
    q2 = inverse_puma(H2,myrobot);

    obs = setupobstacle();
    myrobot = init_robot();
    plotobstacle(obs)
    xlim([-100, 100]);
    ylim([-100, 100]);
    zlim([0, 200]);
    
    %q = motionplan(q1', q2', 1, 1, myrobot, obs, 0.1);
    
    qref = motionplan(q1', q2', 0, 10, myrobot, obs, 0.02);
    t = linspace(0,10,300);
    q = ppval(qref,t)';
    
    plot_robot(q, myrobot)
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