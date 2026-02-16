function setPose(obj, opts)
    arguments
        obj (1,1) anim.rearchassis;
        opts.Yaw (1,1) double = 0;
        opts.PosX (1,1) double = 0;
        opts.PosY (1,1) double = 0;
        opts.Camber (1,1) double = 0;
        opts.RearPitch (1,1) double = 0;
        opts.Pitch (1,1) double = 0;
        opts.Steer (1,1) double = 0;
        opts.FrontPitch (1,1) double = 0;
    end

    args = reshape([fields(opts),struct2cell(opts)]',1,[]);
    setPose(obj.SwingArm, args{:});
    setPose(obj.Base, args{:});
    setPose(obj.Body, args{:});
    setPose(obj.Driver, args{:});
    setPose(obj.LeftArm, args{:});
    setPose(obj.LeftLeg, args{:});
    setPose(obj.RightArm, args{:});
    setPose(obj.RightLeg, args{:});
end