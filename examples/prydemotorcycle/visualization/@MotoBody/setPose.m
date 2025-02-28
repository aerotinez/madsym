function setPose(obj,A)
    arguments
        obj (1,1) MotoBody;
        A (4,4) double;
    end
    p = obj.Vertices;
    T = rigidtform3d(A);
    obj.Patch.Vertices = transformPointsForward(T,p);
end