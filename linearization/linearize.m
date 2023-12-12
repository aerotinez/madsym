function eoml = linearize(strategy,equations,coordinates,constraints,inputs,trim_point)
    arguments
        strategy (1,1) function_handle;
        equations (1,1) EquationsOfMotion;
        coordinates (1,1) GeneralizedCoordinates;
        constraints (1,1) Constraints;
        inputs (:,1) sym = sym.empty(0,1);
        trim_point (1,1) TrimPoint = TrimPoint.empty(0,1);
    end
    eoml = strategy(coordinates,constraints,inputs).linearize(equations,trim_point);