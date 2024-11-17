function M = prydeMotorcycleMassMatrix(in1)
%prydeMotorcycleMassMatrix
%    M = prydeMotorcycleMassMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    15-Nov-2024 22:28:31

%   states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz V Z_f Z_r a a_f0 a_n a_r0 b e f f_w0 f_w2 g h i_fy i_ry k_alpha_f k_alpha_r k_kappa_f k_kappa_r k_varphi_f k_varphi_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
I_bxx = in1(2,:);
I_bxz = in1(3,:);
I_bzz = in1(4,:);
I_hxx = in1(5,:);
I_hzz = in1(6,:);
a = in1(10,:);
b = in1(14,:);
e = in1(15,:);
f = in1(16,:);
h = in1(20,:);
i_fy = in1(21,:);
i_ry = in1(22,:);
m_b = in1(29,:);
m_h = in1(30,:);
rho_r = in1(32,:);
t_r = in1(34,:);
varepsilon = in1(35,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = b.*m_b;
t5 = b.*m_h;
t6 = e.*m_h;
t7 = h.*m_b;
t8 = m_b+m_h;
t9 = m_b.*rho_r;
t10 = m_h.*rho_r;
t11 = m_b.*t_r;
t12 = m_h.*t_r;
t13 = a.^2;
t16 = f.^2;
t19 = varepsilon.*2.0;
t20 = cos(t19);
t21 = t2.^2;
t22 = sin(t19);
t23 = t3.^2;
t24 = I_hzz.*t2;
t25 = I_hzz.*t3;
t26 = e.*t5;
t27 = h.*t4;
t28 = rho_r.*t4;
t29 = rho_r.*t5;
t30 = t4.*t_r;
t31 = t5.*t_r;
t32 = rho_r.*t6;
t33 = t6.*t_r;
t34 = a.*m_h.*t2;
t35 = t2.*t6;
t36 = f.*m_h.*t2;
t37 = a.*m_h.*t3;
t38 = t3.*t6;
t39 = f.*m_h.*t3;
t40 = -t7;
t42 = f.*t2.*t5;
t44 = a.*t3.*t5;
t45 = a.*t2.*t10;
t46 = a.*t2.*t12;
t52 = f.*t3.*t10;
t53 = f.*t3.*t12;
t41 = a.*t35;
t43 = f.*t35;
t47 = a.*t38;
t48 = t3.*t26;
t49 = t2.*t32;
t50 = t2.*t33;
t51 = f.*t38;
t54 = -t25;
t55 = -t27;
t56 = -t36;
t57 = -t37;
t58 = -t38;
t59 = -t39;
t60 = e.*t35;
t62 = -t42;
t64 = -t44;
t68 = -t52;
t69 = -t53;
t70 = a.*f.*m_h.*t20;
t71 = f.*t6.*t20;
t72 = a.*t6.*t22;
t73 = a.*f.*m_h.*t22;
t74 = f.*t6.*t22;
t76 = (I_hxx.*t22)./2.0;
t77 = (I_hzz.*t22)./2.0;
t82 = (m_h.*t13.*t22)./2.0;
t83 = (e.*t6.*t22)./2.0;
t84 = (m_h.*t16.*t22)./2.0;
t63 = -t43;
t65 = -t47;
t66 = -t48;
t67 = -t51;
t75 = e.*t58;
t78 = -t70;
t79 = -t71;
t80 = -t72;
t81 = -t77;
t85 = -t82;
t86 = -t83;
t87 = t4+t5+t34+t35+t59;
t89 = t9+t10+t11+t12+t40+t56+t57+t58;
t88 = t24+t26+t41+t60+t67;
t90 = t32+t33+t54+t63+t65+t75;
t91 = I_bxz+t28+t29+t30+t31+t45+t46+t49+t50+t55+t62+t64+t66+t68+t69+t76+t78+t79+t80+t81+t84+t85+t86;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_bzz-t73-t74+I_hxx.*t23+I_hzz.*t21+b.*t4+b.*t5+t2.*t26.*2.0+a.*t2.*t5.*2.0+a.*t6.*t21.*2.0+e.*t6.*t21-f.*t3.*t5.*2.0+m_h.*t13.*t21+m_h.*t16.*t23,0.0,t87,t91,0.0,t88,0.0,0.0,0.0,0.0,t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t87,0.0,t8,t89,0.0,t6,0.0,0.0,0.0,t91,0.0,t89,I_bxx+t73+t74+I_hxx.*t21+I_hzz.*t23+h.*t7-rho_r.*t7.*2.0+rho_r.*t9+rho_r.*t10-t3.*t32.*2.0-t3.*t33.*2.0-t7.*t_r.*2.0+t9.*t_r.*2.0+t10.*t_r.*2.0+t11.*t_r+t12.*t_r-a.*t3.*t10.*2.0-a.*t3.*t12.*2.0+a.*t6.*t23.*2.0+e.*t6.*t23-f.*t2.*t10.*2.0-f.*t2.*t12.*2.0+m_h.*t13.*t23+m_h.*t16.*t21,0.0,t90,0.0];
mt2 = [0.0,0.0,0.0,0.0,0.0,0.0,i_ry,0.0,0.0,0.0,0.0,t88,0.0,t6,t90,0.0,I_hzz+e.*t6,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,i_fy];
M = reshape([mt1,mt2],9,9);
end
