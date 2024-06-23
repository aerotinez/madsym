function applyMoment(obj,frame,moment)
    arguments
        obj (1,1) Body;
        frame (1,1) Frame;
        moment (3,1) sym;
    end
    obj.applyWrench(frame,obj.MassCenter,[moment;0;0;0]);
end