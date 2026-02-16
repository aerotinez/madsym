function f = invertedPendulumAndCartBody2DynamicsForcingVector(x,u,p)
   %    states = [x theta v omega]
   %    inputs = [F_x]
   %    params = [I_yy b_theta b_x g l m_c m_p]
   
   f0 = invertedPendulumAndCartBody2DynamicsCentrifugalForcingVector(x,p);
   f1 = invertedPendulumAndCartBody2DynamicsCoriolisForcingVector(x,p);
   f2 = invertedPendulumAndCartBody2DynamicsActiveForcingVector(x,u,p);
   f = f0 + f1 + f2;
   
end
