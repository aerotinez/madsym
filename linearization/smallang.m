function f0 = smallang(f, angs)
    arguments
        f sym
        angs (:,1) sym
    end
    f0 = taylor(f,angs,zeros(size(angs)),'Order',2);
end