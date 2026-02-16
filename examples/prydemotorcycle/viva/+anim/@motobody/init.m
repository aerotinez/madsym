function init(obj, opts)
    arguments
        obj
        opts.Parent (1,1) matlab.graphics.axis.Axes = gca
        opts.FileName (1,1) string
        opts.AngleOffset (1,3) double {mustBeReal, mustBeFinite} = [0,0,0]
        opts.CoordinateOffset (1,3) double {mustBeReal, mustBeFinite} = [0,0,0]
        opts.Color (1,3) double {mustBeInRange(opts.Color, 0, 1)} = [0,0,0]
        opts.Scale (1,3) double = [1,1,1]
        opts.Parameters (1,1) anim.motoparams = anim.motoparams
    end
    args = reshape([fields(opts),struct2cell(opts)]',1,[]);
    args = args(1:end - 2);
    init@anim.body(obj, args{:});
    obj.Parameters = opts.Parameters;
    setPose(obj);
end