function validateConstraints(obj)
    arguments
        obj (1,1) GibbsAppell;
    end
    if ~isequal(size(obj.Constraints.Holonomic,1),obj.States.l)
        msga = "Number of holonomic constraints must match number of ";
        msgb = "dependent coordinates.";
        error(msga + msgb);
    end
    if ~isequal(size(obj.Constraints.Nonholonomic,1),obj.States.m)
        msga = "Number of nonholonomic constraints must match number ";
        msgb = "of dependent speeds.";
        error(msga + msgb);
    end
end