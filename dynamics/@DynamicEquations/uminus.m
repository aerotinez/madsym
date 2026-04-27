function eom = uminus(obj)
    arguments
        obj (1,1) DynamicEquations
    end

    eom = obj;

    eom.SpatialInertia = -obj.SpatialInertia;
    eom.ActiveForces   = -obj.ActiveForces;
    eom.MassMatrix = eom.massMatrix();
    eom.ForcingVector = eom.forcingVector();
end