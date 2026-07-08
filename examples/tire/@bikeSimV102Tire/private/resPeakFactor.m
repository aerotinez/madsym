function Dr = resPeakFactor(obj,a,g,fz,dfz)
    Dr = fz.*obj.RAD_TC.*((obj.P_BIKE_QDZ8 + obj.P_BIKE_QDZ9.*dfz).*g + (obj.P_BIKE_QDZ10 + obj.P_BIKE_QDZ11.*dfz).*g.*abs(g))./sqrt(1 + a.^2);
end