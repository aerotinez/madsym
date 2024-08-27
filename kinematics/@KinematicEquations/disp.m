function disp(obj)
    arguments
        obj (1,1) KinematicEquations;
    end
    disp("Coordinates : ");
    pprint(obj.States.');
    disp("Jacobian : ");
    pprint(obj.Jacobian);
    disp("Speeds : ");
    pprint(obj.Inputs.');
end