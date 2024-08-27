function W = applyWrench(obj,frame,point,wrench)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        wrench (6,1) sym;
    end
    W = Wrench(wrench,Pose(frame,point));
    T = Pose(obj.ReferenceFrame,obj.MassCenter);
    obj.ActiveForces = obj.ActiveForces + W.vector(T);
end