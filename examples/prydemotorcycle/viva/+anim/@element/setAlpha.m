function setAlpha(obj,alpha)
    arguments
        obj
        alpha (1,1) double {mustBeInRange(alpha,0,1)}  = 0
    end

    obj.Alpha = alpha;
end