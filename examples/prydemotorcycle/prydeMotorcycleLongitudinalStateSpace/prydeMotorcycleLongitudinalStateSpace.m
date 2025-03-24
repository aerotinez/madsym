function sys = prydeMotorcycleLongitudinalStateSpace(p)
   %    states = [v_rx omega_r omega_f]
   %    inputs = [tau_r tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
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
