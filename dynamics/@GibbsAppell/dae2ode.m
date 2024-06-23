function f = dae2ode(obj)
    arguments
        obj (1,1) GibbsAppell;
    end

    Mkinv = syminv(obj.Kinematics.MassMatrix);
    fk = obj.Kinematics.ForcingVector;

    Mdinv = syminv(obj.Dynamics.MassMatrix);
    fd = obj.Dynamics.ForcingVector;

    Mvinv = [];
    fv = [];
    if ~isempty(obj.Auxiliary)
        Mvinv = syminv(obj.Auxiliary.MassMatrix);
        fv = obj.Auxiliary.ForcingVector;
    end

    Minv = blkdiag(Mkinv,Mdinv,Mvinv);
    f = Minv*[fk;fd;fv];
end