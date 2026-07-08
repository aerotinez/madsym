function Kx = lonStiffnessFactor(obj,fz,dfz)
    Kx = fz.*(obj.P_BIKE_KX1 + obj.P_BIKE_KX2.*dfz).*exp(obj.P_BIKE_KX3.*dfz);
end