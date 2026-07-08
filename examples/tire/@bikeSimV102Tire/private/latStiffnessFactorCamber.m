function Kyg = latStiffnessFactorCamber(obj,fz,dfz)
    Kyg = (obj.P_BIKE_KY6 + obj.P_BIKE_KY7.*dfz).*fz;
end