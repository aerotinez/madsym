classdef RiderUpperBody < matlab.mixin.SetGet
properties
    Mass;
    Ixx;
    Iyy;
    Izz;
    Ixz;
    Rx;
    Ry;
    Rz;
    ForwardLean;
    LeanAxisHeight;
    CoMOffset;
    CoMHeight;
    Stiffness;
    Damping;
end
end