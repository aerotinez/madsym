function sys = sharpMotorcycleSteadyStateStateSpace(p)
   %    states = [varphi delta omega_psi v_y omega_varphi omega_delta]
   %    inputs = [tau_delta]
   %    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz V Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r varepsilon]
   P = sharpMotorcycleSteadyStatePermutationMatrix();
   M = sharpMotorcycleSteadyStateMassMatrix(p);
   H = sharpMotorcycleSteadyStateForcingMatrix(p);
   G = sharpMotorcycleSteadyStateInputMatrix(p);
   A = P*(M\H);
   B = P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi"
      "delta"
      "omega_psi"
      "v_y"
      "omega_varphi"
      "omega_delta"
   ];

   sys.InputName = [
      "tau_delta"
   ];

   sys.OutputName = [
      "varphi"
      "delta"
      "omega_psi"
      "v_y"
      "omega_varphi"
      "omega_delta"
   ];

end
