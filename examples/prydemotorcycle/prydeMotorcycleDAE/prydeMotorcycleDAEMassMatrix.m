function M = prydeMotorcycleDAEMassMatrix(x,p)
    Mk = prydeMotorcycleKinematicMassMatrix();
    Md1 = prydeMotorcycleBody1DynamicsMassMatrix(x,p);
    Md2 = prydeMotorcycleBody2DynamicsMassMatrix(x,p);
    Md3 = prydeMotorcycleBody3DynamicsMassMatrix(x,p);
    Md4 = prydeMotorcycleBody4DynamicsMassMatrix(x,p);
    Md = Md1 + Md2 + Md3 + Md4;
    Mc = prydeMotorcycleConstraintMassMatrix(x,p);
    M = [
        Mk,zeros(12,12);
        zeros(7,12),Md;
        zeros(5,12),Mc
        ];
end
