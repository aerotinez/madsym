classdef TrimPoint
properties
    q;
    u;
    F;
    u_aux;
end
properties
    q0;
    u0;
    F0;
    u0_aux;
end
methods
function obj = TrimPoint(eomd,q0,u0,options)
    arguments
        eomd (1,1) EquationsOfMotion;
        q0 (:,1);
        u0 (:,1);
        options.Inputs (:,1) double = zeros(size(eomd.Inputs));
        options.AuxiliarySpeeds (:,1) double = zeros(size(eomd.AuxiliarySpeeds));
    end
    obj.q = eomd.eomk.q;
    obj.u = eomd.eomk.u;
    obj.F = eomd.Inputs;
    obj.u_aux = eomd.AuxiliarySpeeds;
    obj.q0 = q0;
    obj.u0 = u0;
    obj.F0 = options.Inputs;
    obj.u0_aux = options.AuxiliarySpeeds; 
end
end
end