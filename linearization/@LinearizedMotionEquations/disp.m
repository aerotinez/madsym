function disp(eoml)
    arguments
        eoml (1,1) LinearizedMotionEquations
    end
    disp("States : ");
    pprint(eoml.States.');
    disp("Mass Matrix : ");
    pprint(eoml.MassMatrix);
    disp("Forcing Matrix : ");
    pprint(eoml.ForcingMatrix);
    disp("Input Matrix : ");
    pprint(eoml.InputMatrix);
    disp("Inputs : ");
    pprint(eoml.Inputs.');
end