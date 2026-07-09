function Fy = latForceCombined(obj,k,a,g,fz,dfz)
    Fy0 = latForcePureSlip(obj,a,g,fz,dfz);
    Byk = latStiffnessFactorCombined(obj,a);
    Fy = cos(obj.P_BIKE_RCY1.*atan(Byk.*k)).*Fy0;
end