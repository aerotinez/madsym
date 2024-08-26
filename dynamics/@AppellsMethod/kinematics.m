function kinematics(obj,kdes)
    arguments
        obj (1,1) AppellsMethod;
        kdes (:,1) sym;
    end
    if obj.States.m + obj.States.l > 0 && isempty(obj.Constraints)
        msga = "Constraints equations must be provided for systems with ";
        msgb = "dependent coordinates.";
        error(msga + msgb);
    end
    if numel(kdes) ~= obj.States.k
        error("There must be as many equations as independent speeds.");
    end

    eomk = [
        kdes;
        obj.Constraints.Velocity
        ];

    q = obj.States.Coordinates;
    u = obj.States.Speeds;
    obj.Kinematics = kinematics(q,eomk,u);
    obj.Kinematics.toODE();
end