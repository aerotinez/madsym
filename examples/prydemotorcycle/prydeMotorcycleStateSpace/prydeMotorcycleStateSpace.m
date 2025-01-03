function sys = prydeMotorcycleStateSpace(p)
   %    states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
   %    inputs = [tau_r tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz V Z_f Z_r a a_f0 a_n a_r0 b e f f_w0 f_w2 g h i_fy i_ry k_alpha_f k_alpha_r k_kappa_f k_kappa_r k_varphi_f k_varphi_r m_b m_h rho_f rho_r t_f t_r varepsilon]
   S = prydeMotorcycleSelectorMatrix();
   P = prydeMotorcyclePermutationMatrix();
   M = prydeMotorcycleMassMatrix(p);
   H = prydeMotorcycleForcingMatrix(p);
   G = prydeMotorcycleInputMatrix(p);
   A = S*P*(M\H)*S.';
   B = S*P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_rx";
      "v_ry";
      "omega_bx";
      "omega_r";
      "omega_delta";
      "omega_f";
   ];

   sys.InputName = [
      "tau_r";
      "tau_delta";
   ];

   sys.OutputName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_rx";
      "v_ry";
      "omega_bx";
      "omega_r";
      "omega_delta";
      "omega_f";
   ];

end
