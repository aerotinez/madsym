function Et = trailCurvatureFactor(obj,a,g,dfz,Bt)
    Et = (obj.P_BIKE_QEZ1 + obj.P_BIKE_QEZ2.*dfz).*(1 + obj.P_BIKE_QEZ5.*g.*(2/pi).*atan(Bt.*obj.P_BIKE_QCZ1.*a));
end