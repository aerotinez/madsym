function disp(obj)
    arguments
        obj (1,1) MotionEquations;
    end
    disp("States : ");
    disp(obj.States);
    disp("Mass matrix : ");
    pprint(obj.MassMatrix);
    disp("Forcing vector : ");
    pprint(obj.ForcingVector);
    if ~isempty(obj.Inputs)
        disp("Inputs : ");
        disp(obj.Inputs);
    end
end