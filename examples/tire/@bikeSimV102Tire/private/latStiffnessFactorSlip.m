function Kya = latStiffnessFactorSlip(obj,g,fz)
    Kya = obj.P_BIKE_PKY1.*obj.P_BIKE_FZ0.*sin(obj.P_BIKE_PKY2.*atan(fz./((obj.P_BIKE_PKY3 + obj.P_BIKE_PKY4.*g.^2).*obj.P_BIKE_FZ0)))./(1 + obj.P_BIKE_PKY5.*g.^2);
end