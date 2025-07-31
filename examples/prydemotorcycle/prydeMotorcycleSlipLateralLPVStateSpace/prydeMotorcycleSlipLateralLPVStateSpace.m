function sys = prydeMotorcycleSlipLateralLPVStateSpace()
   %    states = [varphi delta omega_bz v_ry omega_bx omega_delta beta_r beta_f]
   %    inputs = [tau_delta]
   %    params = [A C_d C_delta C_l C_p I_bxx I_byy I_bxz I_bzz I_hxx I_hzz K_ycf K_ycr K_yvf K_yvr V a a_n b e f f_zf0 f_zr0 g h i_fy i_ry m_b m_f m_h m_r p_Kxf1 p_Kxf2 p_Kyf1 p_Kxf3 p_Kyf2 p_Kyf3 p_Kyf6 p_Kyf7 p_Kxr1 p_Kxr2 p_Kyr1 p_Kxr3 p_Kyr2 p_Kyr3 p_Kyr6 p_Kyr7 q_dzf1 q_dzf2 q_dzf8 q_dzf9 q_dzr1 q_dzr2 q_dzr8 q_dzr9 rho rho_f rho_r sigma_yf sigma_yr t_f t_r varepsilon]

   pnames = {
      'A';
      'C_d';
      'C_delta';
      'C_l';
      'C_p';
      'I_bxx';
      'I_byy';
      'I_bxz';
      'I_bzz';
      'I_hxx';
      'I_hzz';
      'K_ycf';
      'K_ycr';
      'K_yvf';
      'K_yvr';
      'V';
      'a';
      'a_n';
      'b';
      'e';
      'f';
      'f_zf0';
      'f_zr0';
      'g';
      'h';
      'i_fy';
      'i_ry';
      'm_b';
      'm_f';
      'm_h';
      'm_r';
      'p_Kxf1';
      'p_Kxf2';
      'p_Kyf1';
      'p_Kxf3';
      'p_Kyf2';
      'p_Kyf3';
      'p_Kyf6';
      'p_Kyf7';
      'p_Kxr1';
      'p_Kxr2';
      'p_Kyr1';
      'p_Kxr3';
      'p_Kyr2';
      'p_Kyr3';
      'p_Kyr6';
      'p_Kyr7';
      'q_dzf1';
      'q_dzf2';
      'q_dzf8';
      'q_dzf9';
      'q_dzr1';
      'q_dzr2';
      'q_dzr8';
      'q_dzr9';
      'rho';
      'rho_f';
      'rho_r';
      'sigma_yf';
      'sigma_yr';
      't_f';
      't_r';
      'varepsilon';
   };

   sys = lpvss(pnames,@prydeMotorcycleSlipLateralDataFcn);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
      "beta_r";
      "beta_f";
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
      "beta_r";
      "beta_f";
   ];

end
