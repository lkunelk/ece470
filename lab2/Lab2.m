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
    [0, d6,-a6, 0    ]
];

kuka = mykuka(DH);

plot(kuka, [0 pi/2 0 0 pi/2 0])

%% test model
test_q = [pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4]'
H = forward_kuka(test_q, kuka);
plot(kuka, test_q')

%% inverse
q = inverse_kuka(H, kuka);
q(1)/pi*180
.6283/pi*180
180/5
