function f = invertedPendulumAndCartKinematicForcingVector(x,u,p)
   %    states = [x theta]
   %    inputs = [v omega]
   %    params = [I_yy b_theta b_x g l m_c m_p]
   
   f = invertedPendulumAndCartKinematicJacobian(x,p)*u;
   
end
