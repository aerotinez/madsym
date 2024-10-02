function inv_pose = inv(obj)
    arguments
        obj (1,1) Pose;
    end
    R = obj.ReferenceFrame.dcm;
    p = obj.Position.posFrom(); 
    inv_pose = Pose(Frame(R.'),Point(-R.'*p));
end