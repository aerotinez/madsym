function v = noslip(b,r,N)
    arguments
        b Body;
        r (3,1) sym = zeros(3,1,'sym');
        N (1,1) Frame = Frame();  
    end
    R = simplify(expand(b.ReferenceFrame.dcm));
    V = simplify(expand(b.Twist.vector()));
    v = N.dcm.'*([-R*vec2skew(r),R]*V);
end