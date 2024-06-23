function d = posFrom(obj,P,N)
    arguments
        obj (1,1) Point;
        P (1,1) Point = Point();
        N (1,1) Frame = Frame();
    end
    p0 = [obj.x,obj.y,obj.z].';
    p = [P.x,P.y,P.z].';
    d = simplify(expand(N.dcm.'*(p0 - p)));
end