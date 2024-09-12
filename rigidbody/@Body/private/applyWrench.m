function applyWrench(obj,frame,point,wrench)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        wrench (6,1) sym;
    end
    obj.ActiveForces = obj.ActiveForces + Wrench(wrench,Pose(frame,point));
end