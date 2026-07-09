function plot(obj,options)
    arguments
        obj (1,1) bikeSimV102Tire
        options.Ns (1,1) double = 100
        options.SlipRatio (1,2) double = [-1,1]
        options.SlipAngle (1,2) double = [-30,30]
        options.Camber (1,2) double = [-60,60]
        options.WheelSpeed (1,2) double = [-110,110]./3.6
    end

    k = linspace(options.SlipRatio(1),options.SlipRatio(2),options.Ns);
    a = linspace(options.SlipAngle(1), options.SlipAngle(2), options.Ns);
    g = linspace(options.Camber(1), options.Camber(2), options.Ns);
    w = linspace(options.WheelSpeed(1), options.WheelSpeed(2), options.Ns);

    Nfz = 5;
    fz = linspace(100,obj.FZ_MAX,5);

    results = zeros(options.Ns,6,Nfz);

    for i = 1:Nfz
        [Fx,Fy,Fz,Mx,My,Mz] = compute(obj,k,a,g,fz(i),w);
        results(:,1,i) = Fx;
        results(:,2,i) = Fy;
        results(:,3,i) = Fz;
        results(:,4,i) = Mx;
        results(:,5,i) = My;
        results(:,6,i) = Mz;
    end

    fig = figure();
    tl = tiledlayout(3,2);
end