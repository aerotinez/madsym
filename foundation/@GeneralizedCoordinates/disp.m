function disp(obj)
    arguments
        obj (1,1) GeneralizedCoordinates
    end
    pprint(obj.All.');
    if isempty(obj.Dependent)
        return;
    end
    disp("Independent :");
    pprint(obj.Independent.');
    disp("Dependent :");
    pprint(obj.Dependent.');
end