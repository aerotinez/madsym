function Br = resStiffnessFactor(obj,By)
    Br = (obj.P_BIKE_QBZ9 + obj.P_BIKE_QBZ10.*By.*obj.P_BIKE_PCY1);
end