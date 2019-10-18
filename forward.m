function H= forward(joint,myrobot)
    H = myrobot.fkine(joint);
end