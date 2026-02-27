function sys = sharpMotorcycleStateSpace(p)
   %    states = [varphi delta omega_psi v_y omega_varphi omega_delta Y_r Y_f]
   %    inputs = [tau_delta]
   %    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e f g h i_fy i_ry j k m_f m_r r_f r_r sigma varepsilon]
   M = sharpMotorcycleSSMassMatrix(p);
   H = sharpMotorcycleSSForcingMatrix(p);
   G = sharpMotorcycleSSInputMatrix(p);
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
      "varphi";
      "delta";
      "omega_psi";
      "v_y";
      "omega_varphi";
      "omega_delta";
      "Y_r";
      "Y_f";
   ];

   sys.InputName = [
      "tau_delta";
   ];

   sys.OutputName = [
      "varphi";
      "delta";
      "omega_psi";
      "v_y";
      "omega_varphi";
      "omega_delta";
      "Y_r";
      "Y_f";
   ];

end
