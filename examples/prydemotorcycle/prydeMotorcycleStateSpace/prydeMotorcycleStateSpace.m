function sys = prydeMotorcycleStateSpace(p)
   %    states = [varphi delta omega_bz v_rx v_ry omega_bx omega_delta]
   %    inputs = [tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz V Z_f Z_r a a_n b e f g h i_fy i_ry k_alpha_f k_alpha_r k_varphi_f k_varphi_r m_b m_h rho_f rho_r t_f t_r varepsilon]
   P = prydeMotorcyclePermutationMatrix();
   M = prydeMotorcycleMassMatrix(p);
   H = prydeMotorcycleForcingMatrix(p);
   G = prydeMotorcycleInputMatrix(p);
   A = P*(M\H);
   B = P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi"
      "delta"
      "omega_bz"
      "v_rx"
      "v_ry"
      "omega_bx"
      "omega_delta"
   ];

   sys.InputName = [
      "tau_delta"
   ];

   sys.OutputName = [
      "varphi"
      "delta"
      "omega_bz"
      "v_rx"
      "v_ry"
      "omega_bx"
      "omega_delta"
   ];

end
