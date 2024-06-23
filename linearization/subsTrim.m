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
    idx = has(ovals,diff(trim_point.u,sym('t')));
    nvals = [diff(trim_point.x0,sym('t'));trim_point.x0];
    nvals(idx) = 0.*nvals(idx);
    f0 = subs(f,ovals,nvals);
end