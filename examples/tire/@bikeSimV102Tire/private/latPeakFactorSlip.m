function Dy = latPeakFactorSlip(obj,g,fz,dfz)
    Dy = fz.*obj.P_BIKE_DY1.*exp(obj.P_BIKE_DY2.*dfz)./(1 + obj.P_BIKE_DY3.*g.^2);
end