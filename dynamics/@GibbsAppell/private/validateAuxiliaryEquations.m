function validateAuxiliaryEquations(obj,equations)
    arguments
        obj (1,1) GibbsAppell;
        equations (:,1) sym;
    end
    if ~isequal(size(equations,1),obj.States.p)
        msga = "Number of auxiliary equations must match number of ";
        msgb = "auxiliary coordinates.";
        error(msga + msgb);
    end
end