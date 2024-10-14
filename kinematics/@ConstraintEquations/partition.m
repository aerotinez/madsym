function eomc = partition(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    hc = obj.Configuration;
    nhc = obj.Jacobian*obj.States.rate;
    nhc = nhc(numel(hc) + 1:end);
    eomc = ConstraintEquations(partition(obj.States),hc,nhc);
    eomc.Speeds = obj.Speeds;
end