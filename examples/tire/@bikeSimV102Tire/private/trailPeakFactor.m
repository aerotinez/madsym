function Dt = trailPeakFactor(obj,g,fz,dfz)
    Dt = fz.*(obj.RAD_TC./obj.P_BIKE_FZ0).*(obj.P_BIKE_QDZ1 + obj.P_BIKE_QDZ2.*dfz).*(1 + obj.P_BIKE_QDZ3.*abs(g) + obj.P_BIKE_QDZ4.*g.^2);
end