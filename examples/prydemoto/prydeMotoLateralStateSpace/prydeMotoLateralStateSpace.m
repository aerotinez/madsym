function sys = prydeMotoLateralStateSpace(p)
   %    states = [gamma delta omega_bz v_ry omega_bx omega_delta alpha_r alpha_f]
   %    inputs = [tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_yar K_ygf K_ygr K_zaf K_zar K_zgf K_zgr R_f R_r V a_n b e f_d f_l f_zf f_zr g h i_fy i_ry j k l_x m_b m_f m_h m_r omega_f0 omega_r0 p rho_f rho_r sigma_alpha_f sigma_alpha_r sigma_kappa_f sigma_kappa_r t_f t_r tau_p varepsilon]
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
   ];

   sys.Offsets.u = [
      0;
   ];

   sys.StateName = [
      "gamma";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
      "alpha_r";
      "alpha_f";
   ];

   sys.InputName = [
      "tau_delta";
   ];

   sys.OutputName = [
      "gamma";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
      "alpha_r";
      "alpha_f";
   ];

end
