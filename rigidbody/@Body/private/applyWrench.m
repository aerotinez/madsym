function W = applyWrench(obj,frame,point,wrench)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        wrench (6,1) sym;
    end 
    W = Wrench(frame,point,wrench).transform(obj.ReferenceFrame,obj.MassCenter);
    obj.ActiveForces = obj.ActiveForces + W.Vector;
end