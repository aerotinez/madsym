function sys = prydeMotorcycleLateralStateSpace(p)
   %    states = [varphi delta omega_bz v_ry omega_bx omega_delta]
   %    inputs = [tau_r tau_delta]
   %    params = [C_alpha_f C_alpha_r C_delta C_gamma_f C_gamma_r C_kappa_f C_kappa_r I_bxx I_bxz I_bzz I_hxx I_hzz K_alpha_f K_alpha_r K_gamma_f K_gamma_r V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
   S = prydeMotorcycleLateralSelectorMatrix();
   P = prydeMotorcycleLateralPermutationMatrix();
   M = prydeMotorcycleLateralMassMatrix(p);
   H = prydeMotorcycleLateralForcingMatrix(p);
   G = prydeMotorcycleLateralInputMatrix(p);
   A = S*P*(M\H)*S.';
   B = S*P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
   ];

   sys.InputName = [
      "tau_r";
      "tau_delta";
   ];

   sys.OutputName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
   ];

end
