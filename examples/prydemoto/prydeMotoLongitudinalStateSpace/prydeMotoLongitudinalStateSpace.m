function sys = prydeMotoLongitudinalStateSpace(p)
   %    states = [v_rx omega_r omega_f kappa_r kappa_f]
   %    inputs = [tau_r tau_br tau_bf]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_yar K_ygf K_ygr K_zaf K_zar K_zgf K_zgr R_f R_r V a_n b e f_d f_l f_zf f_zr g h i_fy i_ry j k l_x m_b m_f m_h m_r omega_f0 omega_r0 p rho_f rho_r sigma_alpha_f sigma_alpha_r sigma_kappa_f sigma_kappa_r t_f t_r tau_p varepsilon]
   M = prydeMotoLongitudinalSSMassMatrix(p);
   H = prydeMotoLongitudinalSSForcingMatrix(p);
   G = prydeMotoLongitudinalSSInputMatrix(p);
   sys = ss(H,G,eye(size(H)),0);

   sys.E = M;

   sys.Offsets.dx = [
      0;
      0;
      0;
      0;
      0;
   ];

   sys.Offsets.x = [
      p(19,:);
      p(39,:);
      p(38,:);
      kappa_r0;
      kappa_f0;
   ];

   sys.Offsets.u = [
      tau_r0;
      0;
      0;
   ];

   sys.StateName = [
      "v_rx";
      "omega_r";
      "omega_f";
      "kappa_r";
      "kappa_f";
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
      "kappa_r";
      "kappa_f";
   ];

end
