function Kx = lonStiffnessFactor(obj,fz,dfz)
    Kx = fz.*(obj.P_BIKE_PKX1 + obj.P_BIKE_PKX2.*dfz).*exp(obj.P_BIKE_PKX3.*dfz);
end