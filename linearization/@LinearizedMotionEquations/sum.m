function eoml = sum(eoml_list)
    arguments
        eoml_list (:,1) LinearizedMotionEquations
    end
    eoml = eoml_list(1);
    for k = 2:length(eoml_list)
        eoml = eoml + eoml_list(k);
    end
end