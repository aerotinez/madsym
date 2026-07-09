function Dy = latPeakFactorSlip(obj,g,fz,dfz)
    Dy = fz.*obj.P_BIKE_PDY1.*exp(obj.P_BIKE_PDY2.*dfz)./(1 + obj.P_BIKE_PDY3.*g.^2);
end