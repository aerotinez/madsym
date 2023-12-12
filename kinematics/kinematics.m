function eomk = kinematics(strategy,coordinates,equations,constraints,inputs)
    arguments
        strategy (1,1) function_handle;
        coordinates (1,1) GeneralizedCoordinates;
        equations (:,1) sym;
        constraints (1,1) Constraints = Constraints();
        inputs (:,1) sym = sym.empty(0,1);
    end
    eomk = strategy(coordinates,constraints,inputs).kinematics(equations);
end