function setPose(obj, opts)
    arguments
        obj (1,1) anim.rightarm;
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
    Nlean = Npitch*roty(p.RiderLean);

    d = opts.PosX*N(:,1) + ...
        opts.PosY*N(:,2) + ...
        p.RearCrownRadius*N(:,3) + ...
        (p.Wheelbase - p.StepOffset)*Npitch(:,1) - ...
        (p.RiderWidth/2)*Npitch(:,2) + ...
        (p.StepHeight + p.LegHeight + p.RearCrownRadius)*Npitch(:,3) - ...
        (p.RiderDepth/2)*Nlean(:,1) + ...
        p.BackHeight*Nlean(:,3);

    setOrientation(obj, Nlean);
    setPosition(obj, d');
end