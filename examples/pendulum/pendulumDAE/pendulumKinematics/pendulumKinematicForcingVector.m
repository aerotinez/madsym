function f = pendulumKinematicForcingVector(x,u,p)
   %    states = [theta]
   %    inputs = [omega]
   %    params = [b g l m t]
   
   f = pendulumKinematicJacobian(x,p)*u;
   
end
