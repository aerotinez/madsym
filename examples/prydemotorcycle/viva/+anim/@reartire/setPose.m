function setPose(obj, opts)
    arguments
        obj (1,1) anim.reartire;
        opts.Yaw (1,1) double = 0;
        opts.PosX (1,1) double = 0;
        opts.PosY (1,1) double = 0;
        opts.Camber (1,1) double = 0;
        opts.RearPitch (1,1) double = 0;
        opts.Pitch (1,1) double = 0;
        opts.Steer (1,1) double = 0;
        opts.FrontPitch (1,1) double = 0;
    end

    p = obj.Parameters;

    N = eye(3);
    Nyaw = rotz(opts.Yaw);
    Ncamber = Nyaw*rotx(opts.Camber);
    R = Ncamber*roty(opts.RearPitch);

    d = opts.PosX*N(:,1) + ...
        opts.PosY*N(:,2) + ...
        p.RearCrownRadius*N(:,3) + ...
        (p.RearRadius - p.RearCrownRadius)*Ncamber(:,3);

    setOrientation(obj, R);
    setPosition(obj, d');
end