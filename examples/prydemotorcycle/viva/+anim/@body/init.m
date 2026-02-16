function init(obj, opts)
    arguments
        obj
        opts.Parent (1,1) matlab.graphics.axis.Axes = gca
        opts.FileName (1,1) string
        opts.AngleOffset (1,3) double {mustBeReal, mustBeFinite} = [0,0,0]
        opts.CoordinateOffset (1,3) double {mustBeReal, mustBeFinite} = [0,0,0]
        opts.Color (1,3) double {mustBeInRange(opts.Color, 0, 1)} = [0,0,0]
        opts.Scale (1,3) double = [1,1,1]
    end
    
    tr = stlread(opts.FileName);
    ax = obj.Axes;
    
    h = trisurf(tr, "Parent", ax, "FaceAlpha", 0);
    h.Vertices = opts.Scale .* h.Vertices;

    q = deg2rad(opts.AngleOffset);
    T = rigidtform3d(angle2dcm(q(1),q(2),q(3))',opts.CoordinateOffset);
    h.Vertices = transformPointsForward(T,h.Vertices);

    h.EdgeColor = "none";
    h.UserData = h.Vertices;
    
    axes(ax);
    uistack(gca, "bottom");
    
    obj.Handle = h;
    obj.AngleOffset = opts.AngleOffset;
    obj.CoordinateOffset = opts.CoordinateOffset;
    
end
