function setPose(obj, opts)
    arguments
        obj (1,1) anim.fronttire;
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
    Npitch = Ncamber*roty(opts.Pitch);
    Ncaster = Npitch*roty(-p.Caster);
    Nsteer = Ncaster*rotz(opts.Steer);
    Npitchf = Nsteer*roty(opts.FrontPitch);

    d = opts.PosX*N(:,1) + ...
        opts.PosY*N(:,2) + ...
        p.RearCrownRadius*N(:,3) + ...
        (p.RearRadius - p.RearCrownRadius)*Ncamber(:,3) + ...
        p.Wheelbase*Npitch(:,1) + ...
        (p.FrontRadius - p.RearRadius)*Npitch(:,3) - ...
        p.Rake*Npitch(:,1) + ...
        p.Rake*Nsteer(:,1);

    setOrientation(obj, Npitchf);
    setPosition(obj, d');
end