function Byk = latStiffnessFactorCombined(obj,a)
    Byk = obj.P_BIKE_RBY1.*cos(atan(obj.P_BIKE_RBY2.*(a - obj.P_BIKE_RBY3)));
end