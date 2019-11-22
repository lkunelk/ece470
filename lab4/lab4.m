function lab4()
    kuka = init_robot(156);
    kuka_forces = init_robot(0);
    
    %test_plot_sample_traj(kuka)
    
    %test_without_obstacle(kuka_forces, kuka);
    
    test_rep_att_forces(kuka)
    
%     test_obstacle(kuka , a6)
end

% Testing the plotting sample trajectory
function test_plot_sample_traj(myrobot)
    q = sample_traj();
    plot_robot(q, myrobot);
end

function test_without_obstacle(kuka_forces, kuka)
    % only test with attractive forces
    H1(1:3,1:3) = [0 0 1;0 -1 0;1 0 0]; % eul2tr converts ZYZ Euler angles to a hom. tsf. mtx
    H1(1:3,4)= [620; 375; 50]; % This assigns the desired displacement to the hom.tsf.mtx.
    H1(4,:) = [0 0 0 1];
    q1 = inverse_kuka(H1, kuka)
    
    H2(1:3,1:3) = [0 0 1;0 -1 0;1 0 0];
    H2(1:3,4)=[620; -375; 50];
    H2(4,:) = [0 0 0 1];
    q2 = inverse_kuka(H2,kuka)
    
    qref = motionplan(q1, q2, 0, 10, kuka_forces, {}, 0.1);
    t = linspace(0,10,100);
    q = ppval(qref,t)';
    
    plot_robot(q, kuka)
end

function test_rep_att_forces(myrobot)
    % after implementing force from floor and cylinder
    % test if we get expected values
    
    kuka = init_robot(156);
    
    % setup test obstacle
    prepobs{1}.R = 100;
    prepobs{1}.c = [250; 0];
    prepobs{1}.rho0 = 500;
    prepobs{1}.h = 300;
    prepobs{1}.type = 'cyl';
    
    q = [pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6];
    
    plot_robot(q, kuka)
    hold on;
    plotobstacle(prepobs)
    
    % TODO: arnav implement force from cylinder and finish testing
    actual_tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], myrobot, prepobs{1})
    expected_tau = [0.1795 0.9540 0.2353 -0.0344 -0.0344 0.0000]
end

function robot = init_robot(a6)
    %initialize robot
    a1 = 25; % [mm]
    a2 = 315;
    a3 = 35;
    d1 = 400;
    d4 = 365;
    d6 = 161.44;

    DH = [
        [0, d1, a1, pi/2 ]
        [0,  0, a2, 0    ]
        [0,  0, a3, pi/2 ]
        [0, d4,  0,-pi/2 ]
        [0,  0,  0, pi/2 ]
        [0, d6, -a6, 0   ]
    ];

    
    robot = mykuka(DH);
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
    
    robot.plot(q, 'floorlevel', 0, 'workspace', [-1000, 1000, -1000, 1000, 0, 1500])
end