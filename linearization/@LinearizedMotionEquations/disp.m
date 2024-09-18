function disp(eoml)
    arguments
        eoml (1,1) LinearizedMotionEquations
    end
    disp("States : ");
    disp(eoml.States);
    disp("Mass Matrix : ");
    pprint(eoml.MassMatrix);
    disp("Forcing Matrix : ");
    pprint(eoml.ForcingMatrix);
    disp("Input Matrix : ");
    pprint(eoml.InputMatrix);
    disp("Inputs : ");
    disp(eoml.Inputs);
end