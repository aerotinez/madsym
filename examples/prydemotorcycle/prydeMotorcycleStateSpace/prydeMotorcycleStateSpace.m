function sys = prydeMotorcycleStateSpace(p)
   %    states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
   %    inputs = [tau_r tau_br tau_bf tau_delta]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_ycf K_ycr K_yvf K_yvr V a a_n b e f f_z0f f_z0r g h i_fy i_ry m_b m_h p_Kx1f p_Kx2f p_Ky1f p_Kx3f p_Ky2f p_Ky3f p_Ky6f p_Ky7f p_Kx1r p_Kx2r p_Ky1r p_Kx3r p_Ky2r p_Ky3r p_Ky6r p_Ky7r q_Dz1f q_Dz2f q_Dz8f q_Dz9f q_Dz1r q_Dz2r q_Dz8r q_Dz9r rho_f rho_r t_f t_r varepsilon]
   Sx = prydeMotorcycleStateSelectorMatrix();
   P = prydeMotorcyclePermutationMatrix();
   M = prydeMotorcycleMassMatrix(p);
   H = prydeMotorcycleForcingMatrix(p);
   G = prydeMotorcycleInputMatrix(p);
   Su = prydeMotorcycleInputSelectorMatrix();
   A = Sx*P*(M\H)*Sx.';
   B = Sx*P*(M\G)*Su.';
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_rx";
      "v_ry";
      "omega_bx";
      "omega_r";
      "omega_delta";
      "omega_f";
   ];

   sys.InputName = [
      "tau_r";
      "tau_br";
      "tau_bf";
      "tau_delta";
   ];

   sys.OutputName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_rx";
      "v_ry";
      "omega_bx";
      "omega_r";
      "omega_delta";
      "omega_f";
   ];

end
