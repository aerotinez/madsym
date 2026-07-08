function Fx0 = lonForcePureSlip(obj,k,fz,dfz)
    D = lonPeakFactor(obj,fz,dfz);
    E = lonCurvatureFactor(obj,k,dfz);
    K = lonStiffnessFactor(obj,fz,dfz);
    C = obj.P_BIKE_PCX1;
    B = K./(C.*D);
    Fx0 = D.*sin(C.*atan(B.*k - E.*(B.*k - atan(B.*k))));
end