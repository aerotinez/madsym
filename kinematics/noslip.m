function v = noslip(b,r,N)
arguments
    b Body;
    r (3,1) sym = zeros(3,1,'sym');
    N (1,1) Frame = Frame();  
end
v = simplify(expand(N.dcm.'*([-b.R*vec2skew(r),b.R]*b.V)));