function M = pendulumDAEMassMatrix(x,p)
    Mk = pendulumKinematicMassMatrix();
    Md1 = pendulumBody1DynamicsMassMatrix(x,p);
    Md = Md1;
    M = [
        Mk,zeros(1,1);
        zeros(1,1),Md;
        ];
end
