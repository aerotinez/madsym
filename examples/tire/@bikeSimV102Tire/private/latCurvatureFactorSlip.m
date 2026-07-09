function Ey = latCurvatureFactorSlip(obj,a,g)
    Ey = obj.P_BIKE_PEY1 + obj.P_BIKE_PEY2.*g.^2 + obj.P_BIKE_PEY4.*sign(a);
end