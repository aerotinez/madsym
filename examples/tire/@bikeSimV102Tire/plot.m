function plot(obj,options)
    arguments
        obj (1,1) bikeSimV102Tire
        options.Ns (1,1) double = 1000
        options.SlipRatio (1,2) double = [-1,1]
        options.SlipAngle (1,2) double = [-15,15]
        options.Camber (1,2) double = [-60,60]
        options.WheelSpeed (1,2) double = ([-110,110]./3.6)./obj.RRE
    end

    k = linspace(options.SlipRatio(1),options.SlipRatio(2),options.Ns)';
    a = linspace(options.SlipAngle(1), options.SlipAngle(2), options.Ns)';
    g = linspace(options.Camber(1), options.Camber(2), options.Ns)';
    w = linspace(options.WheelSpeed(1), options.WheelSpeed(2), options.Ns)';

    Nfz = 5;
    fz = linspace(obj.P_BIKE_FZ0,obj.FZ_MAX,Nfz);

    fig = figure("Position",[720,240,640,480]);

    tl = tiledlayout(3,2, ...
        'Parent',fig, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    x = [100*k,a,a,g,w,a];

    titles = [
        "Driving/Braking";
        "Side";
        "Normal";
        "Overturning";
        "Rolling resistance";
        "Self-aligning"
        ];

    xunits = [
        "ratio (%)";
        "slip angle (\circ)";
        "slip angle (\circ)";
        "camber angle (\circ)";
        "speed (\circ/s)";
        "slip angle (\circ)"
        ];

    yunits = [
        repelem("force (kN)",3).';
        "torque (kNm)";
        "torque (Nm)";
        "torque (Nm)"
        ];

    d2r = pi/180;

    for i = 1:Nfz
        [Fx,Fy,Fz,Mx,My,Mz] = compute(obj,k,d2r*a,d2r*g,fz(i),d2r*w);
        results = [[Fx,Fy,Fz]./1E03,Mx./1E03,My,Mz];
        for j = 1:6
            ax = nexttile(tl,j);
            hold(ax,'on');
            plot(ax,x(:,j),results(:,j),'LineWidth',1);
            hold(ax,'off');
            box(ax,'on');
            axis(ax,'tight');
            title(ax,titles(j),'FontSize',12);
            xlabel(ax,xunits(j),'FontSize',12);
            ylabel(ax,yunits(j),'FontSize',12);
        end
    end

    sgtitle("Magic formula (BikeSim v1.02)",'Parent',tl,'FontSize',12);
    str = arrayfun(@(i)num2str(round(i)) + " N",fz,'UniformOutput',false);
    leg = legend(str{:},'FontSize',12);
    title(leg,"Applied normal force","FontSize",12);
    leg.Orientation = "horizontal";
    leg.Layout.Tile = 'south';
end