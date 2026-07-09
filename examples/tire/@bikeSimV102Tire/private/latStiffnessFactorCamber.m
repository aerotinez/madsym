function Kyg = latStiffnessFactorCamber(obj,fz,dfz)
    Kyg = (obj.P_BIKE_PKY6 + obj.P_BIKE_PKY7.*dfz).*fz;
end