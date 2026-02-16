function f = pendulumBody1DynamicsForcingVector(x,u,p)
   %    states = [theta omega]
   %    inputs = [M_y]
   %    params = [b g l m t]
   
   f0 = pendulumBody1DynamicsCentrifugalForcingVector(x,p);
   f1 = pendulumBody1DynamicsCoriolisForcingVector(x,p);
   f2 = pendulumBody1DynamicsActiveForcingVector(x,u,p);
   f = f0 + f1 + f2;
   
end
