function new_wrench = transform(obj,reference_frame,position)
    arguments
        obj (1,1) Wrench;
        reference_frame (1,1) Frame;
        position (1,1) Point;
    end
    T = Pose(obj.ReferenceFrame,obj.Position);
    Tnew = Pose(reference_frame,position);
    W = simplify(expand(Tnew.Adjoint.'*T.inv().Adjoint.'))*obj.Vector;
    new_wrench = Wrench(reference_frame,position,W);
end