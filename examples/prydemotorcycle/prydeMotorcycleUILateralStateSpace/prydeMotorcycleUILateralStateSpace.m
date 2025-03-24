function sys = prydeMotorcycleUILateralStateSpace(p)
   %    states = [varphi delta omega_bz v_ry omega_bx omega_delta]
   %    inputs = [tau_r tau_delta df_yr df_yf dm_zr dm_zf]
   %    params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
   S = prydeMotorcycleUILateralSelectorMatrix();
   P = prydeMotorcycleUILateralPermutationMatrix();
   M = prydeMotorcycleUILateralMassMatrix(p);
   H = prydeMotorcycleUILateralForcingMatrix(p);
   G = prydeMotorcycleUILateralInputMatrix(p);
   A = S*P*(M\H)*S.';
   B = S*P*(M\G);
   sys = ss(A,B,eye(size(A)),0);

   sys.StateName = [
      "varphi";
      "delta";
      "omega_bz";
      "v_ry";
      "omega_bx";
      "omega_delta";
   ];

   sys.InputName = [
      "tau_r";
      "tau_delta";
      "df_yr";
      "df_yf";
      "dm_zr";
      "dm_zf";
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
