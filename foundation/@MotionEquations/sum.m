function eomd = sum(eomd_list)
    arguments
        eomd_list (:,1) MotionEquations;
    end
    eomd = eomd_list(1);
    for k = 2:numel(eomd_list)
        eomd = eomd + eomd_list(k);
    end
end