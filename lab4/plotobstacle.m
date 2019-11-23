function plotobstacle(obs)

    for i=1:length(obs)
        switch obs{i}.type
            case 'cyl'
                r = obs{i}.R;
                c = obs{i}.c;
                h = obs{i}.h;
                [x,y,z] = cylinder(r,20);                
                x = x + c(1)*ones(size(x));
                y = y + c(2)*ones(size(y));
                z = z*h;
                surf(x,y,z);
            case 'wsp'
                [x,y] = meshgrid(0:0.1:100); 
                z = obs{i}.h + zeros(size(x,1));
                surf(x,y,z);
                
        end
        xlim([-1000, 1000]);
        ylim([-1000, 1000]);
        zlim([0, 2000])
        hold on;
    end

end