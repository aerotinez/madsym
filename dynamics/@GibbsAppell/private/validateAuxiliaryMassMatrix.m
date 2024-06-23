function validateAuxiliaryMassMatrix(obj,mass_matrix)
    arguments
        obj (1,1) GibbsAppell;
        mass_matrix sym;
    end
    if has(mass_matrix,obj.States.Speeds.Independent)
        msga = "Auxiliary mass matrix cannot depend on independent ";
        msgb = "speeds.";
        error(msga + msgb);
    end
end