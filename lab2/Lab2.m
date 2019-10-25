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
n = 100
start_q = [0 pi/2 0 0 0 pi/2 0];
desir_q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4];
%desir_q = [0 pi/2 0 0 pi/2 0]

qs = zeros(n, 6);
for i=1:6
    qs(:, i) = linspace(start_q(i), desir_q(i), n)';
end

H = forward_kuka(desir_q, kuka)
plot(kuka, qs);

%% inverse
%H = [0 0 1 500; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
%H_d = SE3.check(H)
q = inverse_kuka(H, kuka);
desir_q
q'