function setPose(obj, opts)
    arguments
        obj (1,1) anim.frontchassis;
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
    setPose(obj.HandleBars, args{:});
    setPose(obj.ForkTop, args{:});
    setPose(obj.Fork, args{:});
    setPose(obj.FrontFender, args{:});
end