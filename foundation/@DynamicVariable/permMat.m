function P = permMat(obj)
    arguments
        obj (:,1) DynamicVariable;
    end
    P = [
        obj.permMatInd;
        obj.permMatDep
        ];
end