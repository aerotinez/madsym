function sys = prydeMotorcycleLongitudinalStateSpace(p)
   %    states = [v_rx omega_r omega_f]
   %    inputs = [tau_r tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz Psi_alpha_f Psi_alpha_r Psi_varphi_f Psi_varphi_r V Y_alpha_f Y_alpha_r Y_varphi_f Y_varphi_r Z_f Z_r a a_n b e f g h i_fy i_ry k_kappa_f k_kappa_r m_b m_h rho_f rho_r t_f t_r varepsilon]
   S = prydeMotorcycleLongitudinalSelectorMatrix();
   P = prydeMotorcycleLongitudinalPermutationMatrix();
   M = prydeMotorcycleLongitudinalMassMatrix(p);
   H = prydeMotorcycleLongitudinalForcingMatrix(p);
   G = prydeMotorcycleLongitudinalInputMatrix(p);
   A = S*P*(M\H)*S.';
   B = S*P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "v_rx";
      "omega_r";
      "omega_f";
   ];

   sys.InputName = [
      "tau_r";
      "tau_delta";
   ];

   sys.OutputName = [
      "v_rx";
      "omega_r";
      "omega_f";
   ];

end
