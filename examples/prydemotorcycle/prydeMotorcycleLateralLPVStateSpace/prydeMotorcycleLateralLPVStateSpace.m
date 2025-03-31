function sys = prydeMotorcycleLateralLPVStateSpace()
   %    states = [varphi delta omega_bz v_ry omega_bx omega_delta]
   %    inputs = [tau_r tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ycf K_ygf K_yar K_ycr K_ygr K_yvf K_yvr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]

   pnames = {
      'C_delta';
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
      'm_h';
      'rho_f';
      'rho_r';
      't_f';
      't_r';
      'varepsilon';
   };

   sys = lpvss(pnames,@prydeMotorcycleLateralDataFcn);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
   ];

   sys.InputName = [
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
