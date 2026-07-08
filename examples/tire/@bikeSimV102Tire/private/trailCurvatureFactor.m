function Et = trailCurvatureFactor(obj,a,g,dfz,Bt)
    Et = (obj.P_BIKE_EZ1 + obj.P_BIKE_EZ2.*dfz).*(1 + obj.P_BIKE_EZ5.*g.*(2/pi).*atan(Bt.*obj.P_BIKE_QCZ1.*a));
end