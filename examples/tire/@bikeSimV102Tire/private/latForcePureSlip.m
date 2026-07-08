function Fy0 = latForcePureSlip(obj,a,g,fz,dfz)
    Dy = latPeakFactorSlip(obj,g,fz,dfz);
    Ey = latCurvatureFactorSlip(obj,a,g);
    Kya = latStiffnessFactorSlip(obj,g,fz);
    Kyg = latStiffnessFactorCamber(obj,fz,dfz);
    Eg = obj.P_BIKE_PEY5;
    Cy = obj.P_BIKE_PCY1;
    Cg = obj.P_BIKE_PCY2;
    By = Kya./(Cy.*Dy);
    Bg = Kyg./(Cg.*Dy);
    Fy0 = Dy.*sin(Cy.*atan(By.*a - Ey.*(By.*a - atan(By.*a))) + Cg.*atan(Bg.*g - Eg.*(Bg.*g - atan(Bg.*g))));
end