function M = prydeMotorcycleMassMatrix(in1)
%prydeMotorcycleMassMatrix
%    M = prydeMotorcycleMassMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    09-Jan-2025 04:43:21

%   states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz Psi_alpha_f Psi_alpha_r Psi_varphi_f Psi_varphi_r V Y_alpha_f Y_alpha_r Y_varphi_f Y_varphi_r Z_f Z_r a a_f0 a_n a_r0 b e f f_w0 f_w2 g h i_fy i_ry k_kappa_f k_kappa_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
I_bxx = in1(2,:);
I_bxz = in1(3,:);
I_bzz = in1(4,:);
I_hxx = in1(5,:);
I_hzz = in1(6,:);
a = in1(18,:);
a_n = in1(20,:);
b = in1(22,:);
e = in1(23,:);
f = in1(24,:);
h = in1(28,:);
i_fy = in1(29,:);
i_ry = in1(30,:);
m_b = in1(33,:);
m_h = in1(34,:);
rho_f = in1(35,:);
rho_r = in1(36,:);
t_f = in1(37,:);
t_r = in1(38,:);
varepsilon = in1(39,:);
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
t35 = -a_n;
t20 = cos(t19);
t21 = t2.^2;
t22 = sin(t19);
t23 = t3.^2;
t24 = I_hzz.*t2;
t25 = I_hzz.*t3;
t26 = e.*t5;
t27 = h.*t4;
t28 = b.*t2;
t29 = rho_r.*t4;
t30 = rho_r.*t5;
t31 = t4.*t_r;
t32 = t5.*t_r;
t33 = rho_r.*t6;
t34 = t6.*t_r;
t36 = a.*m_h.*t2;
t37 = t2.*t6;
t38 = f.*m_h.*t2;
t39 = a.*m_h.*t3;
t40 = t3.*t6;
t41 = f.*m_h.*t3;
t42 = -t7;
t43 = 1.0./t2;
t45 = f.*t2.*t5;
t47 = a.*t3.*t5;
t48 = a.*t2.*t10;
t49 = a.*t2.*t12;
t55 = f.*t3.*t10;
t56 = f.*t3.*t12;
t44 = a.*t37;
t46 = f.*t37;
t50 = a.*t40;
t51 = t3.*t26;
t52 = t2.*t33;
t53 = t2.*t34;
t54 = f.*t40;
t57 = -t25;
t58 = -t27;
t59 = -t38;
t60 = -t39;
t61 = -t40;
t62 = -t41;
t63 = e.*t37;
t65 = -t45;
t67 = -t47;
t71 = -t55;
t72 = -t56;
t73 = a.*f.*m_h.*t20;
t74 = f.*t6.*t20;
t75 = a.*t6.*t22;
t76 = a.*f.*m_h.*t22;
t77 = f.*t6.*t22;
t79 = (I_hxx.*t22)./2.0;
t80 = (I_hzz.*t22)./2.0;
t81 = a+t28+t35;
t86 = (m_h.*t13.*t22)./2.0;
t87 = (e.*t6.*t22)./2.0;
t88 = (m_h.*t16.*t22)./2.0;
t66 = -t46;
t68 = -t50;
t69 = -t51;
t70 = -t54;
t78 = e.*t61;
t82 = -t73;
t83 = -t74;
t84 = -t75;
t85 = -t80;
t89 = -t86;
t90 = -t87;
t91 = t43.*t81;
t92 = t4+t5+t36+t37+t62;
t94 = t9+t10+t11+t12+t42+t59+t60+t61;
t93 = t24+t26+t44+t63+t70;
t95 = t33+t34+t57+t66+t68+t78;
t96 = I_bxz+t29+t30+t31+t32+t48+t49+t52+t53+t58+t65+t67+t69+t71+t72+t79+t82+t83+t84+t85+t88+t89+t90;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,I_bzz-t76-t77+I_hxx.*t23+I_hzz.*t21+b.*t4+b.*t5+t2.*t26.*2.0+a.*t2.*t5.*2.0+a.*t6.*t21.*2.0+e.*t6.*t21-f.*t3.*t5.*2.0+m_h.*t13.*t21+m_h.*t16.*t23,0.0,t92,t96,0.0,t93,0.0,1.0,0.0,0.0,-t91,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt3 = [0.0,0.0,0.0,0.0,0.0,t92,0.0,t8,t94,0.0,t6,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t96,0.0,t94,I_bxx+t76+t77+I_hxx.*t21+I_hzz.*t23+h.*t7-rho_r.*t7.*2.0+rho_r.*t9+rho_r.*t10-t3.*t33.*2.0-t3.*t34.*2.0-t7.*t_r.*2.0+t9.*t_r.*2.0+t10.*t_r.*2.0+t11.*t_r+t12.*t_r-a.*t3.*t10.*2.0-a.*t3.*t12.*2.0+a.*t6.*t23.*2.0+e.*t6.*t23-f.*t2.*t10.*2.0-f.*t2.*t12.*2.0+m_h.*t13.*t23+m_h.*t16.*t21,0.0,t95,0.0,0.0,-1.0,0.0,rho_f-rho_r+t_f-t_r,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,i_ry,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t93,0.0,t6,t95,0.0,I_hzz+e.*t6,0.0,t2,t3,0.0];
mt4 = [a_n-rho_f.*t3-t3.*t_f,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,i_fy,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t7-t9-t10-t11-t12+t38+t39+t40,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-rho_f+rho_r-t_f+t_r,0.0,t91,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0];
M = reshape([mt1,mt2,mt3,mt4],24,24);
end
