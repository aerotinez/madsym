function J = partitionJacobian(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    x = obj.States;
    if ~isempty(obj.Speeds)
        x = obj.Speeds;
    end
    B = obj.Jacobian;
    Bind = simplify(expand(B(:,1:numel(x.independent))));
    Bdep = simplify(expand(B(:,numel(x.independent) + 1:end)));
    A = syminv(Bdep);
    J = -A*Bind;
end