function Bt = trailStiffnessFactor(obj,g,dfz)
    Bt = (obj.P_BIKE_QBZ1 + obj.P_BIKE_QBZ2.*dfz).*(1 + obj.P_BIKE_QBZ5.*abs(g) + obj.P_BIKE_QBZ6.*g.^2);
end