function equations(obj)
    arguments
        obj (1,1) GibbsAppell;
    end 
    Mk = obj.Kinematics.MassMatrix;
    Md = obj.Dynamics.MassMatrix;
    M = blkdiag(Mk,Md); 

    fk = obj.Kinematics.ForcingVector;
    fd = obj.Dynamics.ForcingVector;
    f = [fk;fd];
    
    if ~isempty(obj.Auxiliary)
        Mv = obj.Auxiliary.MassMatrix;
        M = blkdiag(M,Mv);
        fv = obj.Auxiliary.ForcingVector;
        f = [f;fv];
    end

    x = obj.LinearizationStates;
    obj.Equations = MotionEquations(x,M,f,obj.Inputs);
end