function Ey = latCurvatureFactorSlip(obj,a,g)
    Ey = obj.P_BIKE_EY1 + obj.P_BIKE_EY2.*g.^2 + obj.P_BIKE_EY4.*sign(a);
end