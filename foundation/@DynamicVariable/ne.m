function res = ne(xa,xb)
    arguments
        xa (:,1) DynamicVariable;
        xb (:,1) DynamicVariable;
    end
    res = ~(xa == xb);
end