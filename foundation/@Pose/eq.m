function tf = eq(Ta,Tb)
    arguments
        Ta (1,1) Pose;
        Tb (1,1) Pose;
    end
    Na = Ta.ReferenceFrame;
    Nb = Tb.ReferenceFrame;
    Pa = Ta.Position;
    Pb = Tb.Position;
    tf = eq(Na,Nb) & eq(Pa,Pb);
end