function J = partitionJacobian(obj)
    arguments
        obj (1,1) ConstraintEquations
    end
    x = obj.States;
    if ~isempty(obj.Speeds)
        x = obj.Speeds;
    end
    B = obj.Jacobian;
    Bind = B(:,1:numel(x.independent));
    Bdep = B(:,numel(x.independent) + 1:end);
    J = -syminv(Bdep)*Bind;
end