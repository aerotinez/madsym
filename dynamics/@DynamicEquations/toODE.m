function toODE(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    Minv = syminv(obj.MassMatrix);
    obj.MassMatrix = eye(size(obj.MassMatrix),"sym");
    obj.ForcingVector = Minv*(obj.ForcingVector);
    obj.f0 = Minv*(obj.f0);
    obj.f1 = Minv*(obj.f1);
    obj.f2 = Minv*(obj.f2);
end