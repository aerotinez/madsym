function P = locateNew(obj,p)
    arguments
        obj (1,1) Point;
        p (3,1) sym = zeros(3,1,'sym');
    end
    P = Point(simplify(expand([obj.x,obj.y,obj.z].' + p)));
end