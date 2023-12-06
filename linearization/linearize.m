function eom_lin = linearize(eomd,trim_point,options)
arguments
    eomd (1,1) EquationsOfMotion;
    trim_point (1,1) TrimPoint;
    options.DependentCoordinates (:,1) sym = sym.empty;
end
eoml = Linearizer(eomd,trim_point, ...
    "DependentCoordinates",options.DependentCoordinates);
eom_lin.P = eoml.P;
eom_lin.M = eoml.M;
eom_lin.F = eoml.F;
eom_lin.G = eoml.G;