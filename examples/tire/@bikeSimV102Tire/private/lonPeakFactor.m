function Dx = lonPeakFactor(obj,fz,dfz)
    Dx = (obj.P_BIKE_DX1 + obj.P_BIKE_DX2.*dfz).*fz;
end