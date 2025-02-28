function centerGeometry(obj)
    arguments
        obj (1,1) MotoBody;
    end
    scale = [
        obj.AnimatorParameters.ScaleFactor.X;
        obj.AnimatorParameters.ScaleFactor.Y;
        obj.AnimatorParameters.ScaleFactor.Z;
        ].';

    obj.Vertices = scale.*obj.Vertices;
    obj.Patch.Vertices = obj.Vertices;
    
    angs = [
        obj.AnimatorParameters.AngleOffset.Z;
        obj.AnimatorParameters.AngleOffset.Y;
        obj.AnimatorParameters.AngleOffset.X
        ];

    angs = deg2rad(angs);
    R = angle2dcm(angs(1),angs(2),angs(3)).';

    d = [
        obj.AnimatorParameters.CoordinateOffset.X;
        obj.AnimatorParameters.CoordinateOffset.Y;
        obj.AnimatorParameters.CoordinateOffset.Z
        ].';

    T = rigidtform3d(R,d);

    obj.Vertices = transformPointsForward(T,obj.Vertices);
    obj.Patch.Vertices = obj.Vertices;
end