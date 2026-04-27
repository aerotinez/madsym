function J = constrainingJacobian(obj)
    arguments
        obj (1,1) DynamicEquations
    end
    x = obj.States;
    B = obj.ConstraintJacobian;
    Bind = simplify(expand(B(:,1:numel(x.independent))));
    Bdep = simplify(expand(B(:,numel(x.independent) + 1:end)));
    A = syminv(Bdep);
    J = -A*Bind;
end