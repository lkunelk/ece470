%% define variables
a1 = 25; % [mm]
a2 = 315;
a3 = 35;
a6 = 296.23;
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

kuka = mykuka(DH);
kuka_wrist = mykuka(DH(1:4,:))

plot(kuka, [0 0 0 0 0 0])

%% test model
n = 100;
start_q = [0 pi/2 0 0 0 pi/2 0];
desir_q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4];
%desir_q = [0 pi/2 0 0 pi/2 0]

qs = zeros(n, 6);
for i=1:6
    qs(:, i) = linspace(start_q(i), desir_q(i), n)';
end

H = forward_kuka(qs, kuka);
plot(kuka, qs);

%% test the inverse kinematics
% prints problem if angles from inverse don't match expected angles
for i=1:n
    q = inverse_kuka(H(i), kuka);
    norm(qs(i,:) - q');
    if norm(qs(i,:) - q') > .01;
        q'
        qs(i,:)
        "problem!"
    end
end

%% inverse
%H = [0 0 1 500; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
%H_d = SE3.check(H)
q = inverse_kuka(H, kuka);
desir_q
q'

%%
getAngles()
moveAxis()

%% 
x1 = [624.80; 114.96; 27.24]
q1 = [0.2229    0.7639   -0.2211    0.2580    1.0505   -0.1311]

%%
x2 = [628.84; -146.32; 28.82]
q2 =  [-0.3339; 0.7376; -0.1930; -0.3742; 1.1095 ;0.1890]

%% 
x3 = [484.14; -4.13; 28.83]
q3 = [-0.0463; 1.0081; -0.7894; -0.0466; 1.4034; 0.0084]

%%
delta = fminunc(@deltajoint,[0 0])
myrobot = mykuka_search(delta)
%%
setHome(0.04)
%% Change the z ourselves manually by 2
H = [0 0 1 621.67 ; 0 -1 0 114.96 ; 1 0 0 25 ; 0 0 0 1]
q = inverse_kuka(H,myrobot)
setAngles(q, 0.04)

%%
p_workspace = [600; 100; 10]; 
p_baseframe = FrameTransformation(p_workspace)

R = [0 0 1; 0 -1 0; 1 0 0]
H = [R p_baseframe; zeros(1,3) 1]
q = inverse_kuka(H, myrobot)
setAngles(q, 0.04)
%%
xcoord = linspace(600,600,100);
line = mysegment(xcoord);
R = [0 0 1 ; 0 -1 0; 1 0 0];
H = zeros(4,4);
q = zeros(100,6);
for i = 1:100
    p_baseframe = FrameTransformation(line(:,i))
    H = [R p_baseframe ; 0 0 0 1];
    q(i,:) = inverse_kuka(H, myrobot);
    setAngles(q(i,:),0.04)
end
%plot(myrobot,q)
%%
setHome(0.04)
%%
circle = mycircle();
R = [0 0 1 ; 0 -1 0; 1 0 0];
H = zeros(4,4);
q = zeros(100,6);
for i = 1:100
    p_baseframe = FrameTransformation(circle(:,i))
    H = [R p_baseframe ; 0 0 0 1];
    q(i,:) = inverse_kuka(H, myrobot);
    setAngles(q(i,:),0.04)
end
%%
data = xlsread('jug.xlsx')
xdata = 550 + 10*data(:,1)
ydata = 10*data(:,2);
zdata = 2*(-ones(length(data),1));
R = [0 0 1 ; 0 -1 0; 1 0 0];
H = zeros(4,4);
q = zeros(100,6);
for i = 1:100
    p_baseframe = FrameTransformation([xdata(i);ydata(i);zdata(i)])
    H = [R p_baseframe ; 0 0 0 1];
    q(i,:) = inverse_kuka(H, myrobot);
    setAngles(q(i,:),0.04)
end
%%
myeight = eight()
R = [0 0 1 ; 0 -1 0; 1 0 0];
H = zeros(4,4);
q = zeros(200,6);
for i = 1:200
    p_baseframe = FrameTransformation(myeight(:,i));
    H = [R p_baseframe ; 0 0 0 1];
    q(i,:) = inverse_kuka(H, myrobot);
    setAngles(q(i,:),0.04)
end