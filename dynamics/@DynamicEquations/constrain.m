function eom = constrain(obj,Jc)
    arguments
        obj (1,1) DynamicEquations;
        Jc sym;
    end
    eom_ind = independentDynamics(obj,Jc);
    eom_dep = dependentDynamics(obj,Jc);
    eom = eom_ind + eom_dep;
end

function eom_ind = independentDynamics(eom,Jc)
    n = numel(eom.States);
    m = size(Jc,1);
    k = n - m;
    M = eom.MassMatrix(1:k,:);
    f0 = eom.f0(1:k);
    f1 = eom.f1(1:k);
    f2 = eom.f2(1:k);
    eom_ind = DynamicEquations(eom.States,M,f0,f1,f2,eom.Inputs);
end

function eom_dep = dependentDynamics(eom,Jc)
    m = size(Jc,2);
    M = Jc.'*eom.MassMatrix(m + 1:end,:);
    f0 = Jc.'*eom.f0(m + 1:end);
    f1 = Jc.'*eom.f1(m + 1:end);
    f2 = Jc.'*eom.f2(m + 1:end);
    eom_dep = DynamicEquations(eom.States,M,f0,f1,f2,eom.Inputs);
end