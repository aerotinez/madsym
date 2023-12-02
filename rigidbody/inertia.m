function I = inertia(Ixx,Iyy,Izz,options)
arguments
    Ixx (1,1) sym = sym(1);
    Iyy (1,1) sym = sym(1);
    Izz (1,1) sym = sym(1);
    options.Ixy (1,1) sym = sym(0);
    options.Ixz (1,1) sym = sym(0);
    options.Iyz (1,1) sym = sym(0);
end
I = [
    Ixx, options.Ixy, options.Ixz;
    options.Ixy, Iyy, options.Iyz;
    options.Ixz, options.Iyz, Izz
    ];