function W = applyWrench(obj,frame,point,wrench)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        wrench (6,1) sym;
    end
    W = Wrench(wrench,Pose(frame,point));
    obj.ActiveForces = Wrench(obj.ActiveForces.vector() + W.vector());
end