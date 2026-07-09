function Bxa = lonStiffnessFactorCombined(obj,k)
    Bxa = obj.P_BIKE_RBX1.*cos(atan(obj.P_BIKE_RBX2.*k));
end