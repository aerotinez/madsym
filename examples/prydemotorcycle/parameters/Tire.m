classdef Tire < matlab.mixin.SetGet
    properties
        EffectiveRollingRadius;
        SpringRate;
        Mass;
        UndeflectedCrownRadius;
        SpinInertia;
        Pacejka;
        RollingResistanceCoefficient;
        RollingResistanceTimeConstant;
        LongitudinalRelaxationLength;
        LateralRelaxationLength;
    end
end