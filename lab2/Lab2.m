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
test_q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]'
test_q = [0 0 pi/2 0 pi/2 0]';
H = forward_kuka(test_q, kuka);
H_wrist = forward_kuka(test_q(1:4), kuka_wrist);
plot(kuka, test_q');

%% inverse
q = inverse_kuka(H, kuka);
test_q'
q'