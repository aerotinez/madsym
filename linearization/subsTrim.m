function f0 = subsTrim(f,trim_point)
    arguments
        f sym;
        trim_point (1,1) TrimPoint = TrimPoint.empty();
    end
    if isempty(trim_point)
        f0 = f;
        return
    end
    ovals = [diff(trim_point.x,sym('t'));trim_point.x];
    nvals = [diff(trim_point.x0,sym('t'));trim_point.x0];
    f0 = subs(f,ovals,nvals);
end