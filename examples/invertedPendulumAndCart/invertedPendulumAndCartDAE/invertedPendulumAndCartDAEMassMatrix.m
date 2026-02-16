function M = invertedPendulumAndCartDAEMassMatrix(x,p)
    Mk = invertedPendulumAndCartKinematicMassMatrix();
    Md1 = invertedPendulumAndCartBody1DynamicsMassMatrix(x,p);
    Md2 = invertedPendulumAndCartBody2DynamicsMassMatrix(x,p);
    Md = Md1 + Md2;
    M = [
        Mk,zeros(2,2);
        zeros(2,2),Md;
        ];
end
