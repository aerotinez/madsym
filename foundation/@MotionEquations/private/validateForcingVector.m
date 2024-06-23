function validateForcingVector(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    n = numel(obj.States);
    if ~isequal(size(obj.ForcingVector,1),n)
        msga = "Forcing vector must have as many rows as there ";
        msgb = "are states.";
        error(msga + msgb);
    end
end