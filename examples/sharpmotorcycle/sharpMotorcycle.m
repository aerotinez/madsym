function x_dot = sharpMotorcycle(x,u,p)
%    states = [x y psi varphi delta theta_r psi_f varphi_f theta_f u_1 u_2 u_3 u_4 u_5 Y_r Y_f]
%    inputs = [tau]
%    params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_ry I_rz Z_f Z_r a a_n b e f g h i_fy i_ry m_f m_r r_f r_r sigma varepsilon]
M = sharpMotorcycleMassMatrix(x,u,p);
f = sharpMotorcycleForcingVector(x,u,p);
x_dot = M\f;
end