function eomk = kinematics(strategy,coordinates,equations,constraints)
    arguments
        strategy (1,1) function_handle;
        coordinates (1,1) GeneralizedCoordinates;
        equations (:,1) sym;
        constraints (1,1) Constraints = Constraints();
    end
    K = strategy(coordinates,equations,constraints);
    eomk = K.kinematicEquations();
end