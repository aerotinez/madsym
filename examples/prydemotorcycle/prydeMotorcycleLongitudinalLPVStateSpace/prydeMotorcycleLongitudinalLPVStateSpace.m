function sys = prydeMotorcycleLongitudinalLPVStateSpace()
   %    states = [v_rx omega_r omega_f]
   %    inputs = [tau_r tau_br tau_bf]
   %    params = [A C_d C_delta C_l C_p I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_f m_h m_r rho rho_f rho_r t_f t_r varepsilon]

   pnames = {
      'A';
      'C_d';
      'C_delta';
      'C_l';
      'C_p';
      'I_bxx';
      'I_bxz';
      'I_bzz';
      'I_hxx';
      'I_hzz';
      'K_xkf';
      'K_xkr';
      'K_yaf';
      'K_ycf';
      'K_ygf';
      'K_yar';
      'K_ycr';
      'K_ygr';
      'K_yvf';
      'K_yvr';
      'K_zaf';
      'K_zgf';
      'K_zar';
      'K_zgr';
      'V';
      'a';
      'a_n';
      'b';
      'e';
      'f';
      'g';
      'h';
      'i_fy';
      'i_ry';
      'm_b';
      'm_f';
      'm_h';
      'm_r';
      'rho';
      'rho_f';
      'rho_r';
      't_f';
      't_r';
      'varepsilon';
   };

   sys = lpvss(pnames,@prydeMotorcycleLongitudinalDataFcn);

   sys.StateName = [
      "v_rx";
      "omega_r";
      "omega_f";
   ];

   sys.InputName = [
      "tau_r";
      "tau_br";
      "tau_bf";
   ];

   sys.OutputName = [
      "v_rx";
      "omega_r";
      "omega_f";
   ];

end
