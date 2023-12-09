function v = noslip(b,r,N)
arguments
    b Body;
    r (3,1) sym = zeros(3,1,'sym');
    N (1,1) Frame = Frame();  
end
R = b.ReferenceFrame.dcm;
V = Twist(Pose(b.ReferenceFrame,b.MassCenter)).Vector;
v = N.dcm.'*([-R*vec2skew(r),R]*V);