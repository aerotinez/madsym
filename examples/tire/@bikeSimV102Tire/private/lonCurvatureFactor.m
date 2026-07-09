function Ex = lonCurvatureFactor(obj,k,dfz)
    Ex = (obj.P_BIKE_PEX1 + obj.P_BIKE_PEX2.*dfz + obj.P_BIKE_PEX3.*dfz.^2).*(1 - sign(k));
end