function Lab1()
    my_robot = init_robot();
    q = sample_traj();
    plot_robot(q, my_robot);
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
    hold on % keep the previous plot
    plot(robot,q)
end

function test_inverse()
    % Inverse Kinematics - test sample point
    H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
    inverse(H, robot)
end

function plot_inverse()
    %% Inverse Kinematics - Setup Variables
    x = linspace(10 , 30 , 100);
    y = linspace(23 , 30 , 100);
    z = linspace(15 , 100 ,100);
    o = [x' y' z'];

    R = rotz(pi/4);

    angles = zeros(100, 6);
    for d = 1:100
        H = [R o(d,:)'; 0 0 0 1];
        angles(d,:) = inverse(H, robot);
    end

    %% Inverse Kinemtics - Plot Sample Trajectory
    plot3(o(:,1), o(:,2), o(:,3))
    plot(robot, angles)
    hold on;
end