function [Fx,Fy,Fz,Mx,My,Mz] = compute(obj,k,a,g,fz,w)
    arguments
        obj (1,1) bikeSimV102Tire
        k (:,1) double {mustBeReal}
        a (:,1) double {mustBeReal}
        g (:,1) double {mustBeReal}
        fz (:,1) double {mustBeReal}
        w (:,1) double {mustBeReal}
    end
    Fz = max(fz.*ones(size(k)),zeros(size(k)));
    dfz = fzRatio(obj,Fz);
    Fx = lonForceCombined(obj,k,a,Fz,dfz);
    Fy = latForceCombined(obj,k,a,g,Fz,dfz);
    w = w.*ones(size(k));
    My = rollingResistance(obj,w,Fz);
    Mz = aligningMomentCombined(obj,k,a,g,Fz,dfz);
    Mx = overturningMoment(obj,g,Fx,Fy,Fz,My,Mz);
end
