function auxiliary(obj,equations)
    arguments
        obj (1,1) GibbsAppell;
        equations (:,1) sym;
    end
    obj.validateAuxiliaryEquations(equations);
    qd = diff(obj.States.Coordinates.All);
    fk = obj.Kinematics.ForcingVector;
    eq = subs(equations,qd,fk);
    v = obj.States.Auxiliary;
    vd = diff(v);
    M = jacobian(eq,vd);
    obj.validateAuxiliaryMassMatrix(M);
    f = -subs(eq,vd,0.*vd);
    obj.Auxiliary = MotionEquations(v,M,f,obj.Inputs);
end