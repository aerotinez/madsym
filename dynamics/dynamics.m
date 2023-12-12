function eomd = dynamics(strategy,coordinates,kinematics,bodies,constraints,inputs)
    arguments
        strategy function_handle;
        coordinates GeneralizedCoordinates;
        kinematics KinematicEquations;
        bodies Body;
        constraints (1,1) Constraints = Constraints();
        inputs (:,1) sym = sym.empty(0,1);
    end
    fB = @(b)b.dynamics(strategy,coordinates,kinematics,constraints,inputs);
    B = arrayfun(fB,bodies);

    q = B(1).q;
    u = B(1).u;
    v = B(1).v;
    M = sum(cell2sym(reshape({B.MassMatrix},1,1,[])),3);
    f = sum([B.ForcingVector],2);

    sum3 = @(f)sum(cell2sym(reshape(arrayfun(f,B,'uniform',0),1,1,[])),3);
    x = [q;u;v];
    Ml = sum3(@(b)b.Linearized.MassMatrix);
    Hl = sum3(@(b)b.Linearized.ForcingMatrix);
    Gl = sum3(@(b)b.Linearized.InputMatrix);
    eoml = LinearizedEquations(x,Ml,Hl,Gl);

    eomd = DynamicEquations(q,u,v,M,f,eoml);
end 
