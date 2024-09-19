function sys = sharpMotorcycleStateSpace(x,p)
   %    states = [varphi delta omega_psi omega_varphi omega_delta v_y Y_r Y_f]
   %    inputs = [tau_delta]
   %    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r sigma v_x varepsilon]
   A = sharpMotorcycleStateMatrix(x,p);
   B = sharpMotorcycleInputMatrix(x,p);
   sys = ss(A,B,eye(size(A)),0);
end
