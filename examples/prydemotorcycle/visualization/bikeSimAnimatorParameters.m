function p = bikeSimAnimatorParameters(ang,scale,coord,ref)
    arguments
        ang (1,3);
        scale (1,3);
        coord (1,3);
        ref (1,3);
    end
    p.AngleOffset = helper(ang);
    p.ScaleFactor = helper(scale);
    p.CoordinateOffset = helper(coord);
    p.ReferenceLength = helper(ref);
end

function p = helper(val)
    p.X = val(1);
    p.Y = val(2);
    p.Z = val(3);
end