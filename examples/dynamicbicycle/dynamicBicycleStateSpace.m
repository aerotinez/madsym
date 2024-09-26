function sys = dynamicBicycleStateSpace(x,p)
   %    states = [omega_z v_x v_y]
   %    inputs = [delta a_x]
   %    params = [C_f C_r I_zz V l_f l_r m]
   A = dynamicBicycleStateMatrix(x,p);
   B = dynamicBicycleInputMatrix(x,p);
   sys = ss(A,B,eye(size(A)),0);
end
