function Dx = lonPeakFactor(obj,fz,dfz)
    Dx = (obj.P_BIKE_PDX1 + obj.P_BIKE_PDX2.*dfz).*fz;
end