function disp(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    disp("Coordinates : ");
    disp(obj.States);
    disp("Jacobian : ");
    pprint(obj.Jacobian);
    disp("Speeds : ");
    disp(obj.Inputs);
end