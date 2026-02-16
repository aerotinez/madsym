function transform(obj, opts)
    arguments
        obj (1,:) cell;
        opts.Orientation (3,3) double = obj{1}.Orientation;
        opts.Position (1,3) double = obj{1}.Position;
    end

    fps = 60;
    ts = 1/fps;
    tf = 0.25;

    ns = tf / ts;

    N = numel(obj);

    R = obj{1}.Orientation;
    p = obj{1}.Position;

    [yaw0,pitch0,roll0] = dcm2angle(R');
    [yawf,pitchf,rollf] = dcm2angle(opts.Orientation');

    yaw = rad2deg(linspace(yaw0,yawf,ns));
    pitch = rad2deg(linspace(pitch0,pitchf,ns));
    roll = rad2deg(linspace(roll0,rollf,ns));

    px = linspace(p(1),opts.Position(1),ns);
    py = linspace(p(2),opts.Position(2),ns);
    pz = linspace(p(3),opts.Position(3),ns);

    for k = 1:ns
        for i = 1:N
            R = rotz(yaw(k))*roty(pitch(k))*rotx(roll(k));
            p = [px(k),py(k),pz(k)];
            setPose(obj{i},"Orientation",R,"Position",p);
        end
        pause(ts);
    end

end