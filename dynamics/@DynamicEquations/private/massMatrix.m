function M = massMatrix(obj)
    M = obj.Jacobian.' * obj.SpatialInertia * obj.Jacobian;

    if numel(dependent(obj.States)) > 0
        Jc = obj.constrainingJacobian();
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        M = M(1:k,:) + Jc.'*M(k+1:end,:);
    end
end