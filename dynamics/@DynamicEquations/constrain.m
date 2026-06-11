function eom = constrain(obj,Jc)
    arguments
        obj (1,1) DynamicEquations
        Jc sym
    end
    eom = obj;
    eom.ConstraintJacobian = Jc;
    eom.MassMatrix = eom.massMatrix();
    eom.ForcingVector = eom.forcingVector();
end