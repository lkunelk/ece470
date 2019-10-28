function line = mycircle()
    theta = linspace(0,2*pi,100);
    radius = 50;
    xcoord = 620 + 50*cos(theta); 
    ycoord = 50*sin(theta);
    zcoord = linspace(-2,-2,100);
    line = [xcoord ; ycoord; zcoord]