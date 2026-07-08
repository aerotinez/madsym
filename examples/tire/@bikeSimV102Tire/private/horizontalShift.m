function SHr = horizontalShift(obj,g,dfz)
    SHr = (obj.P_BIKE_QHZ3 + obj.P_BIKE_QHZ4.*dfz).*g;
end