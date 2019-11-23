function lab4()
    kuka = init_robot(156);
    kuka_forces = init_robot(0);
    %find_cyl_pos()
    %init_motion_planning();
    init_creative_motion_planning();
    %test_plot_sample_traj(kuka)
    
    %test_without_obstacle(kuka_forces, kuka);
    
    %test_rep_att_forces(kuka_forces, kuka)
    
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

function test_rep_att_forces(kuka_forces, kuka)
    % after implementing force from floor and cylinder
    % test if we get expected values
    
    kuka = init_robot(156);
    
    % setup test obstacle
    prepobs{1}.R = 100;
    prepobs{1}.c = [250; 0];
    prepobs{1}.rho0 = 100;
    prepobs{1}.h = 300; 
    prepobs{1}.type = 'cyl';
    
    prepobs{2}.rho0 = 200;
    prepobs{2}.h = 32; 
    prepobs{2}.type = 'wsp';
    
    actual_tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6], kuka_forces, prepobs{1})
    expected_tau = [0.1795 0.9540 0.2353 -0.0344 -0.0344 0.0000]
    
    p1 = [620 375 50];
    p2 = [620 -375 50];
    R=[0 0 1;0 -1 0;1 0 0];
    H1=[R p1';zeros(1,3) 1];
    H2=[R p2';zeros(1,3) 1];
    
    q1 = inverse_kuka(H1, kuka);
    q2 = inverse_kuka(H2, kuka);
    
    qref = motionplan(q1, q2, 0, 10, kuka_forces, prepobs, 0.02);
    t = linspace(0,10,100);
    q = ppval(qref,t)';
    
    plot_robot(q, kuka)
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
    H = robot.fkine(q);
    m = H.transl;
    plot3(m(:,1),m(:,2),m(:,3),'r')
    
    robot.plot(q, 'floorlevel', 0, 'workspace', [-1000, 1000, -1000, 1000, 0, 1500])
end

function init_motion_planning()
    % 
    z_offset = 17;
    z_grid = 45; 
    p0 = [370; -440; 225 + z_offset];
    p1 = [370; -440; z_grid + z_offset];
    p2 = [750; -220; 225 + z_offset]; 
    p3 = [620; 350; 225 + z_offset];
    
    q_home = [0; 1.5708; 0; 0; 1.5708; 0];
    
    kuka = init_robot(156);
    kuka_forces = init_robot(0);
    
    R = [0 0 1; 0 -1 0; 1 0 0];
    H0 = [R p0 ; 0 0 0 1];
    H1 = [R p1 ; 0 0 0 1];
    H2 = [R p2 ; 0 0 0 1];
    H3 = [R p3 ; 0 0 0 1];
    q0 = inverse_kuka(H0,kuka);
    q1 = inverse_kuka(H1,kuka);
    q2 = inverse_kuka(H2,kuka);
    q3 = inverse_kuka(H3,kuka);
    obs = setupobstacle(); 
    plotobstacle(obs);
    tol = 0.01;
    qref0 = motionplan(q_home,q0,0,10,kuka_forces,obs,tol);
    
    t = linspace(0,10,100);
    q_result0 = ppval(qref0,t)'
    
    %qref1 = motionplan(q0,q1,0,10,kuka_forces,obs,tol)
    %q_result1 = ppval(qref1,t)';
    
    qref2 = motionplan(q1,q2,0,10,kuka_forces,obs,tol);
    q_result2 = ppval(qref2,t)';
    
    
    qref3 = motionplan(q2,q3,0,10,kuka_forces,obs,tol);
    q_result3 = ppval(qref3,t)';
    
    %q_result = [q_result0; q_result1 ; q_result2; q_result3]
    q_result = [q_result0]
    q_result_final = [q_result2; q_result3]
    q_result_total = [q_result0; q_result2; q_result3]
    vel = 0.04;
    for i = 1:length(q_result)
        setAngles(q_result(i,:), vel);
    end
    setGripper(0);
    setAngles(q1,vel);
    setGripper(1);
    for i = 1:length(q_result_final)
        setAngles(q_result_final(i,:), vel);
    end
    setGripper(0);
    %plot_robot(q_result_total, kuka)    
end

function init_creative_motion_planning()
    % 
    z_offset_init = 25;
    z_offset = 5;
    z_grid = 45; 
    p0 = [370; -440; 225 + z_offset_init];
    p1 = [370; -440; z_grid + z_offset_init];
    p2 = [750; -220; 225 + z_offset_init]; 
    p3 = [620; 350; 225 + z_offset_init];
    
    q_home = [0; 1.5708; 0; 0; 1.5708; 0];
    
    kuka = init_robot(156);
    kuka_forces = init_robot(0);
    
    R = [0 0 1; 0 -1 0; 1 0 0];
    H0 = [R p0 ; 0 0 0 1];
    H1 = [R p1 ; 0 0 0 1];
    H2 = [R p2 ; 0 0 0 1];
    H3 = [R p3 ; 0 0 0 1];
    q0 = inverse_kuka(H0,kuka);
    q1 = inverse_kuka(H1,kuka);
    q2 = inverse_kuka(H2,kuka);
    q3 = inverse_kuka(H3,kuka);
    obs = setupobstacle(); 
    plotobstacle(obs);
    tol = 0.01;
    qref0 = motionplan(q_home,q0,0,10,kuka_forces,obs,tol);
    
    t = linspace(0,10,100);
    q_result0 = ppval(qref0,t)'
    
    %qref1 = motionplan(q0,q1,0,10,kuka_forces,obs,tol)
    %q_result1 = ppval(qref1,t)';
    
    qref2 = motionplan(q1,q2,0,10,kuka_forces,obs,tol);
    q_result2 = ppval(qref2,t)';
    
    
    qref3 = motionplan(q2,q3,0,10,kuka_forces,obs,tol);
    q_result3 = ppval(qref3,t)';
    
    %q_result = [q_result0; q_result1 ; q_result2; q_result3]
    q_result = [q_result0]
    q_result_final = [q_result2; q_result3]
    q_result_total = [q_result0; q_result2; q_result3]
    vel = 0.04;
    for i = 1:length(q_result)
        setAngles(q_result(i,:), vel);
    end
    setGripper(0);
    setAngles(q1,vel);
    setGripper(1);
    for i = 1:length(q_result_final)
        setAngles(q_result_final(i,:), vel);
    end
    setGripper(0);
    
    %plot_robot(q_result_total, kuka)    
    
    p0 = [370; -440; 225 + z_offset];
    p1 = [370; -440; z_grid + z_offset];
    p2 = [750; -220; 225 + z_offset]; 
    p3 = [620; 350; 225 + z_offset];
    
    R = [0 0 1; 0 -1 0; 1 0 0];
    H0 = [R p0 ; 0 0 0 1];
    H1 = [R p1 ; 0 0 0 1];
    H2 = [R p2 ; 0 0 0 1];
    H3 = [R p3 ; 0 0 0 1];
    q0 = inverse_kuka(H0,kuka);
    q1 = inverse_kuka(H1,kuka);
    q2 = inverse_kuka(H2,kuka);
    q3 = inverse_kuka(H3,kuka);
    obs = setupobstacle(); 
    plotobstacle(obs);
    tol = 0.01;
    qref0 = motionplan(q_home,q0,0,10,kuka_forces,obs,tol);
    
    t = linspace(0,10,100);
    q_result0 = ppval(qref0,t)'
    
    qref2 = motionplan(q1,q2,0,10,kuka_forces,obs,tol);
    q_result2 = ppval(qref2,t)';
    
    
    qref3 = motionplan(q2,q3,0,10,kuka_forces,obs,tol);
    q_result3 = ppval(qref3,t)';
    
    %q_result = [q_result0; q_result1 ; q_result2; q_result3]
    q_result = [q_result0]
    q_result_final = [q_result2; q_result3]
    %q_result_total = [q_result0; q_result2; q_result3]
    vel = 0.04;
    for i = 1:length(q_result)
        setAngles(q_result(i,:), vel);
    end
    setGripper(0);
    setAngles(q1,vel);
    setGripper(1);
    for i = 1:length(q_result_final)
        setAngles(q_result_final(i,:), vel);
    end
    setGripper(0);
end


function find_cyl_pos()
    z_offset = 10;
    pos1 = [620; 0; 32 + z_offset]; 
    pos2 = [620; -440; 32+z_offset];
    R = [0 0 1; 0 -1 0; 1 0 0];
    H0 = [R pos1 ; 0 0 0 1];
    H1 = [R pos2 ; 0 0 0 1];
    
    kuka = init_robot(156);
    vel = 0.04; 
    setHome(vel);
    q0 = inverse_kuka(H0,kuka);
    q1 = inverse_kuka(H1,kuka);
    
    %setAngles(q0,vel);
    %setHome(vel);
    setAngles(q1, vel);
    
end