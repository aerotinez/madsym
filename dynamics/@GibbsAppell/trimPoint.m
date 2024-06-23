function trimPoint(obj,q0,u0,v0,F0)
    arguments
        obj (1,1) GibbsAppell;
        q0 (:,1) sym;
        u0 (:,1) sym;
        v0 (:,1) sym = sym.empty(0,1);
        F0 (:,1) sym = sym.empty(0,1);
    end
    q = obj.States.Coordinates.All;
    u = obj.States.Speeds.Independent;
    v = obj.States.Auxiliary;
    F = obj.Inputs;
    obj.Trim = TrimPoint(q,u,v,F,q0,u0,v0,F0);
end