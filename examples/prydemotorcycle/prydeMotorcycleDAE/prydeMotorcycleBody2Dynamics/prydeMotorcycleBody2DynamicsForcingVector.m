function f = prydeMotorcycleBody2DynamicsForcingVector(x,u,p)
   %    states = [psi p_x p_y varphi theta_r delta theta_f theta psi_f p_fx p_fy varphi_f omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f omega_by omega_fz v_fx v_fy omega_fx]
   %    inputs = [tau_r tau_br tau_bf tau_delta]
   %    params = [A C_d C_delta C_l C_p I_bxx I_bxz I_byy I_bzz I_hxx I_hzz K_ycf K_ycr K_yvf K_yvr a a_n b e f f_zf0 f_zr0 g h i_fy i_ry m_b m_f m_h m_r p_Kxf1 p_Kxf2 p_Kxf3 p_Kxr1 p_Kxr2 p_Kxr3 p_Kyf1 p_Kyf2 p_Kyf3 p_Kyf6 p_Kyf7 p_Kyr1 p_Kyr2 p_Kyr3 p_Kyr6 p_Kyr7 q_dzf1 q_dzf2 q_dzf8 q_dzf9 q_dzr1 q_dzr2 q_dzr8 q_dzr9 rho rho_f rho_r t_f t_r varepsilon]
   
   f0 = prydeMotorcycleBody2DynamicsCentrifugalForcingVector(x,p);
   f1 = prydeMotorcycleBody2DynamicsCoriolisForcingVector(x,p);
   f2 = prydeMotorcycleBody2DynamicsActiveForcingVector(x,u,p);
   f = f0 + f1 + f2;
   
end
