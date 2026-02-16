function setPosition(obj, p)
    setPosition@anim.element(obj, p);
    obj.Handle.XData = p(1);
    obj.Handle.YData = p(2);
    obj.Handle.ZData = p(3);

    R = obj.Orientation;
    d = obj.Position;
    pf = transformPointsForward(rigidtform3d(R,d),3 * obj.Direction);

    obj.DimensionLine.XData = [p(1),pf(1)];
    obj.DimensionLine.YData = [p(2),pf(2)];
    obj.DimensionLine.ZData = [p(3),pf(3)];
end