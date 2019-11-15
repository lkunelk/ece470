%% Forward Kinematics - setup variables
initial = 0;
final = 2*pi;
N = 200;

theta1 = linspace(0,2*pi,N);
theta2 = linspace(0,pi/2,N);
theta3 = linspace(0,pi,N);
theta4 = linspace(pi/4,3*pi/2,N);
theta5 = linspace(-pi/3,pi/3,N);
theta6 = linspace(0,2*pi,N);
q = [theta1;theta2;theta3;theta4;theta5;theta6].';
DH = [[0, 76, 0, pi/2];[0,-23.65,43.23,0];[0,0,0,pi/2];[0,43.18,0,-pi/2];[0,0,0,pi/2];[0,20,0,0]];
robot = mypuma560(DH);

%% Forward Kinemtaics - Plot robot following sample trajectory
H = forward(q,robot);
m = H.transl
plot3(m(:,1),m(:,2),m(:,3),'r')
hold on
plot(robot,q)
%% Forward Kinemtaics Lab 3 - Plot robot following sample trajectory
Hnew = zeros(200,4,4,6)
for i = 1:200
    Hnew(i,:,:,:) = forward_puma_lab3(q(i,:),robot);
end
mnew = Hnew(:,1:3,4,6)
plot3(mnew(:,1),mnew(:,2),mnew(:,3),'r')
hold on
plot(robot,q)

%% Inverse Kinematics - test sample point
H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
inverse(H, robot)

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