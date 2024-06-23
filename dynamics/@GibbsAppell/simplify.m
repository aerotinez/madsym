function simplify(obj)
    arguments
        obj (1,1) GibbsAppell;
    end
    obj.Kinematics.simplify();
    obj.Dynamics.simplify();
    if ~isempty(obj.Auxiliary)
        obj.Auxiliary.simplify();
    end
end