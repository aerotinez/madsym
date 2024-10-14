function u = speeds(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    u = obj.States.rate;
    if ~isempty(obj.Speeds)
        u = obj.Speeds.state;
    end
end