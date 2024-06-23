function validateMassMatrix(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    n = numel(obj.States);
    if ~isequal(size(obj.MassMatrix,1),size(obj.MassMatrix,2),n)
        msga = "Mass matrix must have as many rows and columns ";
        msgb = "as there are states.";
        error(msga + msgb);
    end
end