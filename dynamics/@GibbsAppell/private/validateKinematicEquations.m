function validateKinematicEquations(obj,equations)
    arguments
        obj (1,1) GibbsAppell;
        equations (:,1) sym;
    end
    if ~isequal(size(equations,1),obj.States.k)
        msga = "Number of kinematic equations must match number of ";
        msgb = "independent speeds.";
        error(msga + msgb);
    end
end