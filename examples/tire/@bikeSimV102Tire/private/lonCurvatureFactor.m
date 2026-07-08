function Ex = lonCurvatureFactor(obj,k,dfz)
    Ex = (obj.P_BIKE_EX1 + obj.P_BIKE_EX2.*dfz + obj.P_BIKE_EX3.*dfz.^2).*(1 - sign(k));
end