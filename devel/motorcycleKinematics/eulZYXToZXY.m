function [yaw,lean,pitch] = eulZYXToZXY(yaw,pitch,roll)
    arguments
        yaw (:,1) double;
        pitch (:,1) double;
        roll (:,1) double;
    end
    q = quaternion([yaw,pitch,roll],"eulerd","ZYX","frame");
    angs = eulerd(q,"ZXY","frame");
    yaw = angs(:,1);
    lean = angs(:,2);
    pitch = angs(:,3);
end