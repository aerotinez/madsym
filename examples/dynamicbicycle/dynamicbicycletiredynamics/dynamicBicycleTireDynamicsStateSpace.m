function sys = dynamicBicycleTireDynamicsStateSpace(x,p)
   %    states = [omega_z v_x v_y F_yr F_yf]
   %    inputs = [delta a_x]
   %    params = [C_f C_r I_zz V l_f l_r m sigma_f sigma_r]
   A = dynamicBicycleTireDynamicsStateMatrix(x,p);
   B = dynamicBicycleTireDynamicsInputMatrix(x,p);
   sys = ss(A,B,eye(size(A)),0);
end
