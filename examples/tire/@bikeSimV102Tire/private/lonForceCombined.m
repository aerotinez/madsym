function Fx = lonForceCombined(obj,k,a,fz,dfz)
    Fx0 = lonForcePureSlip(obj,k,fz,dfz);
    Bxa = lonStiffnessFactorCombined(obj,k);
    Fx = cos(obj.P_BIKE_RCX1.*atan(Bxa.*a)).*Fx0;
end