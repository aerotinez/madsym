function M = massMatrix(obj)
    M = obj.Jacobian.' * obj.SpatialInertia * obj.Jacobian;

    Jc = obj.constrainingJacobian();
    if ~isequal(Jc, eye(numel(obj.States),'sym'))
        n = numel(obj.States);
        m = size(Jc,1);
        k = n - m;

        M = M(1:k,:) + Jc.'*M(k+1:end,:);
    end
end