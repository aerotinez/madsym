classdef Tire < matlab.mixin.SetGet
    properties
        EffectiveRollingRadius;
        SpringRate;
        Mass;
        UndeflectedCrownRadius;
        SpinInertia;
        Pacejka;
    end
end