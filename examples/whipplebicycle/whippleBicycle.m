function x_dot = whippleBicycle(x,u,p)
%    states = [psi varphi theta_r x y theta x_f y_f psi_f varphi_f theta_f beta u_1 u_2 u_3]
%    inputs = [M_th M_delta]
%    params = [I_bx I_bxz I_by I_bz I_fx I_fy I_fz I_hx I_hxz I_hy I_hz I_rx I_ry I_rz b d e f g h m_b m_f m_h m_r p r_f r_r varepsilon]
M = whippleBicycleMassMatrix(x,u,p);
f = whippleBicycleForcingVector(x,u,p);
x_dot = M\f;
end