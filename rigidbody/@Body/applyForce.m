function applyForce(obj,frame,point,force)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        point (1,1) Point;
        force (3,1) sym;
    end
    obj.applyWrench(frame,point,[0;0;0;force]);
end