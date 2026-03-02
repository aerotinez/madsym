function sys = prydeMotoLateralStateSpace(p)
   %    states = [gamma y_w varphi delta omega_bz v_ry omega_bx v_l omega_varphi omega_delta alpha_r alpha_f]
   %    inputs = [f_w tau_varphi tau_delta]
   %    params = [C_delta C_l C_varphi I_bxx I_bxz I_bzz I_hxx I_hzz I_lxx I_lzz I_uxx I_uxz I_uzz K_xkf K_xkr K_yaf K_yar K_ygf K_ygr K_zaf K_zar K_zgf K_zgr R_f R_r V a_n b b_l b_w e f_d f_l f_zf f_zr g h h_l h_u h_w i_fy i_ry j k k_l k_varphi l_x m_b m_f m_h m_l m_r m_u nu omega_f0 omega_r0 p rho_f rho_r sigma_alpha_f sigma_alpha_r sigma_kappa_f sigma_kappa_r t_f t_r tau_p varepsilon]
   M = prydeMotoLateralSSMassMatrix(p);
   H = prydeMotoLateralSSForcingMatrix(p);
   G = prydeMotoLateralSSInputMatrix(p);
   sys = ss(H,G,eye(size(H)),0);

   sys.E = M;

   sys.Offsets.dx = [
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
   ];

   sys.Offsets.x = [
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
      0;
   ];

   sys.Offsets.u = [
      0;
      0;
      0;
   ];

   sys.StateName = [
      "gamma";
      "y_w";
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "v_l";
      "omega_varphi";
      "omega_delta";
      "alpha_r";
      "alpha_f";
   ];

   sys.InputName = [
      "f_w";
      "tau_varphi";
      "tau_delta";
   ];

   sys.OutputName = [
      "gamma";
      "y_w";
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "v_l";
      "omega_varphi";
      "omega_delta";
      "alpha_r";
      "alpha_f";
   ];

end
