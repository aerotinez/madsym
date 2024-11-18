function H = prydeMotorcycleForcingMatrix(in1)
%prydeMotorcycleForcingMatrix
%    H = prydeMotorcycleForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    18-Nov-2024 11:57:49

%   states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz V Z_f Z_r a a_f0 a_n a_r0 b e f f_w0 f_w2 g h i_fy i_ry k_alpha_f k_alpha_r k_kappa_f k_kappa_r k_varphi_f k_varphi_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
C_delta = in1(1,:);
V = in1(7,:);
Z_f = in1(8,:);
Z_r = in1(9,:);
a = in1(10,:);
a_f0 = in1(11,:);
a_n = in1(12,:);
a_r0 = in1(13,:);
b = in1(14,:);
e = in1(15,:);
f = in1(16,:);
f_w0 = in1(17,:);
f_w2 = in1(18,:);
g = in1(19,:);
h = in1(20,:);
i_fy = in1(21,:);
i_ry = in1(22,:);
k_alpha_f = in1(23,:);
k_alpha_r = in1(24,:);
k_kappa_f = in1(25,:);
k_kappa_r = in1(26,:);
k_varphi_f = in1(27,:);
k_varphi_r = in1(28,:);
m_b = in1(29,:);
m_h = in1(30,:);
rho_f = in1(31,:);
rho_r = in1(32,:);
t_f = in1(33,:);
t_r = in1(34,:);
varepsilon = in1(35,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = Z_f.*k_alpha_f;
t5 = Z_r.*k_alpha_r;
t6 = rho_f+t_f;
t7 = rho_r+t_r;
t8 = V.^2;
t9 = a.^2;
t10 = a_n.^2;
t11 = b.^2;
t12 = t_f.^2;
t13 = t_r.^2;
t14 = varepsilon.*2.0;
t15 = varepsilon.*3.0;
t26 = Z_f.*a.*t_f;
t27 = Z_f.*a_n.*t_f;
t33 = 1.0./V;
t34 = -a_n;
t16 = cos(t14);
t17 = t2.^2;
t18 = t2.^3;
t20 = sin(t14);
t21 = sin(t15);
t22 = t3.^2;
t23 = t3.^3;
t24 = a.*t4;
t25 = a_n.*t4;
t28 = rho_r.*t4;
t29 = rho_r.*t5;
t30 = t4.*t_f;
t31 = t4.*t_r;
t32 = b.*t2;
t35 = 1.0./t2;
t38 = f_w2.*t8.*2.0;
t39 = -t27;
t41 = 1.0./t6;
t42 = 1.0./t7;
t43 = t4.*t10;
t45 = t4+t5;
t47 = Z_f.*f_w0.*rho_f.*t2.*t_f;
t48 = Z_r.*f_w0.*rho_r.*t2.*t_f;
t49 = Z_r.*f_w0.*t2.*t_f.*t_r;
t50 = a.*e.*g.*m_h.*t3;
t52 = Z_f.*a_n.*f_w0.*rho_f.*t2.*2.0;
t53 = Z_f.*f_w0.*t2.*t12;
t54 = f_w0.*t2.*t27.*2.0;
t57 = Z_f.*k_kappa_f.*rho_f.*t2.*t_f.*2.0;
t58 = Z_f.*k_kappa_f.*rho_r.*t2.*t_f.*4.0;
t59 = Z_r.*k_kappa_r.*rho_r.*t2.*t_f.*2.0;
t60 = Z_r.*k_kappa_r.*t2.*t_f.*t_r.*2.0;
t67 = Z_f.*k_kappa_f.*t2.*t12.*2.0;
t68 = e.*g.*m_h.*t3.*t34;
t77 = Z_f.*f_w2.*rho_f.*t2.*t8.*t_f;
t78 = Z_r.*f_w2.*rho_r.*t2.*t8.*t_f;
t79 = Z_r.*f_w2.*t2.*t8.*t_f.*t_r;
t84 = Z_f.*f_w2.*t2.*t8.*t12;
t19 = t17.^2;
t36 = t4.*t32;
t37 = Z_f.*t32.*t_f;
t40 = -t30;
t44 = -t23;
t46 = a_f0.*t2.*t25;
t55 = g.*m_b.*t32.*t_f.*2.0;
t56 = g.*m_h.*t32.*t_f.*2.0;
t61 = f.*g.*m_h.*t20.*t_f;
t62 = b.*t25.*t33;
t63 = b.*t28.*t33;
t64 = b.*t31.*t33;
t65 = V.*i_fy.*t41;
t66 = V.*i_ry.*t42;
t71 = -t58;
t72 = -t59;
t73 = -t60;
t74 = a+t32+t34;
t75 = a.*g.*m_h.*t17.*t_f.*2.0;
t76 = e.*g.*m_h.*t17.*t_f.*2.0;
t83 = Z_f.*a_n.*rho_f.*t2.*t38;
t85 = t2.*t27.*t38;
t89 = (b.*e.*g.*m_h.*t20)./2.0;
t91 = a_n.*t24.*t33.*t35;
t92 = rho_r.*t24.*t33.*t35;
t93 = rho_r.*t25.*t33.*t35;
t94 = t24.*t33.*t35.*t_r;
t95 = t25.*t33.*t35.*t_r;
t96 = t33.*t35.*t43;
t69 = -t55;
t70 = -t56;
t80 = t3+t44;
t81 = -t75;
t82 = -t76;
t86 = t3.*t65;
t87 = -t63;
t88 = -t64;
t90 = 1.0./t74;
t97 = -t92;
t98 = -t94;
t99 = -t96;
t100 = t28+t29+t31+t40;
et1 = t26+t37+t39+t47+t48+t49+t50+t53+t57+t61+t67+t68+t69+t70+t71+t72+t73+t77+t78+t79+t81+t82+t84+t89-Z_f.*a.*t_r+Z_f.*a_n.*t_r-Z_f.*t32.*t_r-Z_r.*f_w0.*t2.*t13-a.*g.*m_b.*t_r-a.*g.*m_h.*t_r+a_n.*g.*m_b.*t_r+a_n.*g.*m_h.*t_r+Z_f.*k_varphi_f.*rho_r.*t32+Z_f.*k_varphi_f.*rho_r.*t34+Z_r.*k_varphi_r.*rho_r.*t32+Z_r.*k_varphi_r.*rho_r.*t34+Z_f.*k_kappa_f.*t2.*t13.*4.0+Z_r.*k_kappa_r.*t2.*t13.*2.0+Z_f.*k_varphi_f.*t32.*t_r+Z_f.*k_varphi_f.*t34.*t_r+Z_r.*k_varphi_r.*t32.*t_r+Z_r.*k_varphi_r.*t34.*t_r+g.*h.*m_b.*t32+g.*h.*m_b.*t34+g.*m_h.*t3.*t9+g.*m_b.*t32.*t_r+g.*m_h.*t32.*t_r+Z_f.*a.*k_varphi_f.*rho_r+Z_r.*a.*k_varphi_r.*rho_r+Z_f.*a.*k_varphi_f.*t_r+Z_r.*a.*k_varphi_r.*t_r+a.*g.*h.*m_b+(a.*b.*g.*m_h.*t20)./2.0+a.*f.*g.*m_h.*t2+b.*f.*g.*m_h.*t17;
et2 = -Z_f.*f_w0.*rho_f.*t2.*t_r-Z_r.*f_w0.*rho_r.*t2.*t_r-Z_r.*f_w2.*t2.*t8.*t13+a.*g.*m_h.*t3.*t34-Z_f.*f_w0.*t2.*t_f.*t_r+a.*g.*m_h.*t17.*t_r.*2.0-Z_f.*k_kappa_f.*rho_f.*t2.*t_r.*2.0+Z_f.*k_kappa_f.*rho_r.*t2.*t_r.*4.0+Z_r.*k_kappa_r.*rho_r.*t2.*t_r.*2.0+e.*g.*m_h.*t17.*t_r.*2.0+f.*g.*m_h.*t2.*t34-Z_f.*k_kappa_f.*t2.*t_f.*t_r.*6.0-f.*g.*m_h.*t20.*t_r-Z_f.*f_w2.*rho_f.*t2.*t8.*t_r-Z_r.*f_w2.*rho_r.*t2.*t8.*t_r-Z_f.*f_w2.*t2.*t8.*t_f.*t_r;
et3 = -t52-t54-Z_f.*t9+t3.*t26+t3.*t39+Z_f.*a.*a_n-Z_f.*a.*t32.*2.0+Z_f.*a_n.*t32-Z_f.*t11.*t17-b.*t17.*t28-b.*t17.*t31+f_w0.*t2.*t26-k_kappa_f.*t2.*t26.*2.0-rho_r.*t2.*t24+rho_r.*t2.*t25-t2.*t24.*t_r+t2.*t25.*t_r+(Z_f.*b.*t20.*t_f)./2.0+(Z_f.*f_w0.*t12.*t20)./2.0+a_n.*g.*m_b.*t32.*2.0+a_n.*g.*m_h.*t32.*2.0+e.*g.*m_h.*t32+e.*g.*m_h.*t34+Z_f.*k_kappa_f.*t12.*t20+f_w2.*t2.*t8.*t26-f_w2.*t2.*t8.*t27.*2.0+a.*e.*g.*m_h+Z_f.*a.*f_w0.*rho_f.*t2+Z_f.*b.*f_w0.*rho_f.*t17+a.*a_n.*g.*m_h.*t17.*2.0+Z_f.*b.*f_w0.*t17.*t_f-Z_f.*a.*k_kappa_f.*rho_f.*t2.*2.0+Z_f.*a.*k_kappa_f.*rho_r.*t2.*2.0+Z_f.*a.*k_varphi_f.*rho_r.*t3+Z_f.*a_n.*k_kappa_f.*rho_r.*t2.*2.0;
et4 = Z_r.*a_n.*k_kappa_r.*rho_r.*t2.*2.0-Z_f.*b.*k_kappa_f.*rho_f.*t17.*2.0+Z_f.*b.*k_kappa_f.*rho_r.*t17.*2.0+(Z_f.*b.*k_varphi_f.*rho_r.*t20)./2.0+a_n.*e.*g.*m_h.*t17.*2.0+Z_f.*a.*k_kappa_f.*t2.*t_r.*2.0+Z_f.*a.*k_varphi_f.*t3.*t_r+Z_f.*a_n.*k_kappa_f.*t2.*t_r.*2.0+Z_r.*a_n.*k_kappa_r.*t2.*t_r.*2.0-Z_f.*b.*k_kappa_f.*t17.*t_f.*2.0+Z_f.*b.*k_kappa_f.*t17.*t_r.*2.0+(Z_f.*b.*k_varphi_f.*t20.*t_r)./2.0+Z_r.*f_w0.*rho_r.*t2.*t34+(Z_f.*f_w0.*rho_f.*t20.*t_f)./2.0+(Z_r.*f_w0.*rho_r.*t20.*t_f)./2.0+(Z_f.*f_w2.*t8.*t12.*t20)./2.0+Z_r.*f_w0.*t2.*t34.*t_r+(Z_r.*f_w0.*t20.*t_f.*t_r)./2.0-a.*g.*m_h.*t80.*t_f.*2.0-b.*g.*m_b.*t20.*t_f-b.*g.*m_h.*t20.*t_f+Z_f.*k_varphi_f.*rho_r.*t3.*t34+Z_f.*k_kappa_f.*rho_f.*t20.*t_f-Z_f.*k_kappa_f.*rho_r.*t20.*t_f.*2.0-Z_r.*k_kappa_r.*rho_r.*t20.*t_f-e.*g.*m_h.*t80.*t_f.*2.0;
et5 = Z_f.*k_varphi_f.*t3.*t34.*t_r+f.*g.*m_h.*t20.*t34-Z_f.*k_kappa_f.*t20.*t_f.*t_r.*2.0-Z_r.*k_kappa_r.*t20.*t_f.*t_r+f.*g.*m_h.*t2.*t_f.*2.0-f.*g.*m_h.*t18.*t_f.*2.0+Z_f.*a.*f_w2.*rho_f.*t2.*t8-Z_f.*a_n.*f_w2.*rho_f.*t2.*t8.*2.0+Z_f.*b.*f_w2.*rho_f.*t8.*t17+Z_f.*b.*f_w2.*t8.*t17.*t_f+Z_r.*f_w2.*rho_r.*t2.*t8.*t34+(Z_f.*f_w2.*rho_f.*t8.*t20.*t_f)./2.0+(Z_r.*f_w2.*rho_r.*t8.*t20.*t_f)./2.0+Z_r.*f_w2.*t2.*t8.*t34.*t_r+(Z_r.*f_w2.*t8.*t20.*t_f.*t_r)./2.0;
et6 = t52+t54+t83+t85-t3.*t26.*2.0+t3.*t27.*2.0+Z_f.*k_varphi_f.*t10.*2.0+k_kappa_f.*t2.*t26.*4.0+Z_f.*a.*t3.*t_r.*2.0-Z_f.*a_n.*t3.*t_r.*2.0-Z_f.*b.*t20.*t_f+Z_f.*b.*t20.*t_r-Z_f.*f_w0.*t12.*t20-a_n.*g.*m_b.*t32.*4.0-a_n.*g.*m_h.*t32.*4.0-e.*g.*m_h.*t32.*2.0-Z_f.*k_kappa_f.*t12.*t20.*2.0-Z_f.*a.*a_n.*k_varphi_f.*2.0-Z_f.*a_f0.*b.*k_varphi_f-a.*a_n.*g.*m_h.*2.0+Z_f.*b.*k_kappa_f.*rho_f.*2.0-a.*e.*g.*m_h.*2.0-Z_f.*a_n.*k_varphi_f.*t32.*2.0+Z_f.*b.*k_kappa_f.*t_f.*2.0-Z_f.*a.*a_f0.*k_varphi_f.*t2.*2.0+Z_f.*a_f0.*a_n.*k_varphi_f.*t2.*2.0-Z_f.*a_f0.*b.*k_varphi_f.*t16+Z_r.*a_n.*f_w0.*rho_r.*t2.*2.0+Z_r.*a_n.*f_w0.*t2.*t_r.*2.0-a.*a_n.*g.*m_h.*t16.*2.0+Z_f.*a.*k_kappa_f.*rho_f.*t2.*4.0-Z_f.*a_n.*k_kappa_f.*rho_r.*t2.*8.0-Z_r.*a_n.*k_kappa_r.*rho_r.*t2.*4.0;
et7 = Z_f.*b.*k_kappa_f.*rho_f.*t16.*2.0-a_n.*e.*g.*m_h.*t16.*2.0-Z_f.*a_n.*k_kappa_f.*t2.*t_r.*8.0-Z_r.*a_n.*k_kappa_r.*t2.*t_r.*4.0+a_n.*f.*g.*m_h.*t20.*2.0+Z_f.*b.*k_kappa_f.*t16.*t_f.*2.0+Z_r.*a_n.*rho_r.*t2.*t38+Z_r.*a_n.*t2.*t38.*t_r-Z_f.*f_w0.*rho_f.*t20.*t_f-Z_r.*f_w0.*rho_r.*t20.*t_f-Z_f.*f_w2.*t8.*t12.*t20-Z_r.*f_w0.*t20.*t_f.*t_r+a.*g.*m_h.*t3.*t_f+a.*g.*m_h.*t21.*t_f+b.*g.*m_b.*t20.*t_f.*2.0+b.*g.*m_h.*t20.*t_f.*2.0-Z_f.*k_kappa_f.*rho_f.*t20.*t_f.*2.0+Z_f.*k_kappa_f.*rho_r.*t20.*t_f.*4.0+Z_r.*k_kappa_r.*rho_r.*t20.*t_f.*2.0+e.*g.*m_h.*t3.*t_f+e.*g.*m_h.*t21.*t_f+Z_f.*k_kappa_f.*t20.*t_f.*t_r.*4.0+Z_r.*k_kappa_r.*t20.*t_f.*t_r.*2.0-f.*g.*m_h.*t2.*t_f+f.*g.*m_h.*t_f.*cos(t15)-Z_f.*f_w2.*rho_f.*t8.*t20.*t_f-Z_r.*f_w2.*rho_r.*t8.*t20.*t_f;
et8 = -Z_r.*f_w2.*t8.*t20.*t_f.*t_r;
et9 = t26+t37+t39+t47+t48+t49+t50+t53+t57+t61+t67+t68+t69+t70+t71+t72+t73+t77+t78+t79+t81+t82+t84+t89-t17.*t26+t17.*t27+t2.*t43-Z_f.*t3.*t9-Z_f.*t11.*t80-a_f0.*t17.*t24+a_f0.*t17.*t25-b.*t17.*t25+(f_w0.*t20.*t26)./2.0+f_w0.*t20.*t39-k_kappa_f.*t20.*t26+t2.*t24.*t34-Z_f.*b.*t18.*t_f-Z_f.*f_w0.*t12.*t18-a_f0.*b.*t4.*t18-Z_f.*k_kappa_f.*t12.*t18.*2.0-Z_f.*k_varphi_f.*t3.*t10+(f_w2.*t8.*t20.*t26)./2.0+f_w2.*t8.*t20.*t39+Z_f.*a.*a_n.*t3-Z_f.*a.*b.*t20+(Z_f.*a_n.*b.*t20)./2.0+(Z_f.*a.*a_f0.*k_varphi_f.*t20)./2.0+Z_f.*a.*a_n.*k_varphi_f.*t3-(Z_f.*a_f0.*a_n.*k_varphi_f.*t20)./2.0+Z_f.*a_f0.*b.*k_varphi_f.*t80+(Z_f.*a_n.*b.*k_varphi_f.*t20)./2.0;
et10 = (Z_f.*a.*f_w0.*rho_f.*t20)./2.0-(Z_r.*a_n.*f_w0.*rho_r.*t20)./2.0+Z_f.*b.*f_w0.*rho_f.*t80-(Z_r.*a_n.*f_w0.*t20.*t_r)./2.0+a.*a_n.*g.*m_h.*t80.*2.0+Z_f.*b.*f_w0.*t80.*t_f+a_n.*b.*g.*m_b.*t20+a_n.*b.*g.*m_h.*t20-Z_f.*a.*k_kappa_f.*rho_f.*t20+Z_f.*a_n.*k_kappa_f.*rho_r.*t20.*2.0+Z_r.*a_n.*k_kappa_r.*rho_r.*t20-Z_f.*b.*k_kappa_f.*rho_f.*t80.*2.0+a_n.*e.*g.*m_h.*t80.*2.0+Z_f.*a_n.*k_kappa_f.*t20.*t_r.*2.0+Z_r.*a_n.*k_kappa_r.*t20.*t_r-a_n.*f.*g.*m_h.*t2.*2.0+a_n.*f.*g.*m_h.*t18.*2.0-Z_f.*b.*k_kappa_f.*t80.*t_f.*2.0+Z_f.*f_w0.*rho_f.*t20.*t34-Z_f.*f_w0.*rho_f.*t18.*t_f-Z_r.*f_w0.*rho_r.*t18.*t_f-Z_f.*f_w2.*t8.*t12.*t18-Z_r.*f_w0.*t18.*t_f.*t_r+a.*g.*m_h.*t19.*t_f.*2.0+b.*g.*m_b.*t18.*t_f.*2.0+b.*g.*m_h.*t18.*t_f.*2.0-Z_f.*k_kappa_f.*rho_f.*t18.*t_f.*2.0;
et11 = Z_f.*k_kappa_f.*rho_r.*t18.*t_f.*4.0+Z_r.*k_kappa_r.*rho_r.*t18.*t_f.*2.0+e.*g.*m_h.*t19.*t_f.*2.0-Z_f.*k_kappa_f.*t2.*t_f.*t_r.*4.0+Z_f.*k_kappa_f.*t18.*t_f.*t_r.*4.0+Z_r.*k_kappa_r.*t18.*t_f.*t_r.*2.0+(Z_f.*a.*f_w2.*rho_f.*t8.*t20)./2.0-(Z_r.*a_n.*f_w2.*rho_r.*t8.*t20)./2.0+Z_f.*b.*f_w2.*rho_f.*t8.*t80-(Z_r.*a_n.*f_w2.*t8.*t20.*t_r)./2.0+Z_f.*b.*f_w2.*t8.*t80.*t_f+Z_f.*f_w2.*rho_f.*t8.*t20.*t34-Z_f.*f_w2.*rho_f.*t8.*t18.*t_f-Z_r.*f_w2.*rho_r.*t8.*t18.*t_f-Z_r.*f_w2.*t8.*t18.*t_f.*t_r-f.*g.*m_h.*t3.*t18.*t_f.*2.0;
mt1 = [0.0,0.0,t35.*(Z_f.*a.*k_varphi_f+Z_f.*k_varphi_f.*t32+Z_f.*k_varphi_f.*t34+Z_f.*k_kappa_f.*rho_r.*t2.*2.0+Z_r.*k_kappa_r.*rho_r.*t2.*2.0+Z_f.*k_kappa_f.*t2.*t_r.*2.0+Z_r.*k_kappa_r.*t2.*t_r.*2.0-Z_f.*a_f0.*k_varphi_f.*t2-Z_r.*a_r0.*k_varphi_r.*t2),0.0,Z_f.*k_varphi_f+Z_r.*k_varphi_r,t90.*(et1+et2),0.0,(et6+et7+et8)./(a.*2.0-a_n.*2.0+t32.*2.0),0.0,0.0,0.0,-Z_f.*t35.*(-a.*k_alpha_f.*t2+a.*k_kappa_f.*t2.*2.0+a.*k_varphi_f.*t3+a_f0.*k_alpha_f.*t17+a_n.*k_alpha_f.*t2-(a_f0.*k_varphi_f.*t20)./2.0-b.*k_alpha_f.*t17+b.*k_kappa_f.*t17.*2.0+(b.*k_varphi_f.*t20)./2.0+k_varphi_f.*t3.*t34),0.0,-Z_f.*(-k_alpha_f.*t2+k_kappa_f.*t2.*2.0+k_varphi_f.*t3),-t90.*(et3+et4+et5),0.0,t90.*(et9+et10+et11),0.0,0.0,0.0];
mt2 = [-(t33.*(t43+t46-a_n.*t24.*2.0+t4.*t9+t24.*t32.*2.0-t25.*t32.*2.0-a_f0.*t2.*t24+t4.*t11.*t17-a_f0.*b.*t4.*t17+a.*m_h.*t8.*t18+b.*m_b.*t8.*t17+b.*m_h.*t8.*t17+e.*m_h.*t8.*t18-f.*m_h.*t3.*t8.*t17))./t17,0.0,-t33.*t35.*(t24-t25+t36+m_b.*t2.*t8+m_h.*t2.*t8),-t65-t66+t87+t88+t93+t95+t97+t98+V.*h.*m_b-V.*m_b.*rho_r-V.*m_h.*rho_r-V.*m_b.*t_r-V.*m_h.*t_r+V.*a.*m_h.*t3+V.*e.*m_h.*t3+V.*f.*m_h.*t2,0.0,t62+t86+t91+t99-V.*e.*m_h+a_f0.*t33.*t36+a_f0.*t17.*t24.*t33-a_f0.*t17.*t25.*t33+a_f0.*t22.*t24.*t33-a_f0.*t22.*t25.*t33,0.0,0.0,0.0,0.0];
mt3 = [t33.*(Z_f.*k_kappa_f+Z_r.*k_kappa_r),0.0,0.0,-Z_r.*t7.*t33.*(k_kappa_r+t38),0.0,-Z_f.*t6.*t33.*(k_kappa_f+t38),0.0,0.0,t33.*t35.*(-t24+t25-t36+a_f0.*t2.*t4+a_r0.*t2.*t5),0.0,-t33.*t45,-t7.*t33.*t45,0.0,t4.*t33.*(a_n+a_f0.*t2),0.0,1.0,0.0,t65+t66+t87+t88+t93+t95+t97+t98+a_f0.*t28.*t33+a_f0.*t31.*t33+a_r0.*t29.*t33+a_f0.*rho_f.*t4.*t33-b.*rho_f.*t4.*t33+a_f0.*t17.*t33.*t40+a_f0.*t22.*t33.*t40+b.*t17.*t30.*t33+b.*t22.*t30.*t33+t24.*t33.*t35.*t_f-t25.*t33.*t35.*t_f-a_f0.*rho_f.*t4.*t17.*t33-a_f0.*rho_f.*t4.*t22.*t33+b.*rho_f.*t4.*t17.*t33+b.*rho_f.*t4.*t22.*t33,0.0,-t33.*t100,-t7.*t33.*t100,0.0];
mt4 = [t33.*t41.*(-t12.*t25+i_fy.*t2.*t8+rho_f.*rho_r.*t25-rho_f.*t25.*t_f+rho_r.*t25.*t_f+rho_f.*t25.*t_r+t25.*t_f.*t_r+a_f0.*rho_f.*t2.*t28+a_f0.*rho_f.*t2.*t31+a_f0.*rho_f.*t2.*t40-a_f0.*t2.*t4.*t12+a_f0.*t2.*t28.*t_f+a_f0.*t2.*t30.*t_r),0.0,0.0,0.0,0.0,Z_r.*k_kappa_r.*t7.*t33,0.0,0.0,-Z_r.*k_kappa_r.*t33.*1.0./t42.^2,0.0,0.0,0.0,1.0,t62-t86+t91+t99-a_f0.*t25.*t33+a_f0.*t3.*t30.*t33+b.*t3.*t33.*t40-t3.*t24.*t33.*t35.*t_f+t3.*t25.*t33.*t35.*t_f,0.0,t4.*t33.*(a_n-t3.*t_f),-t2.*t65+rho_r.*t25.*t33+t25.*t33.*t_r-t3.*t28.*t33.*t_f+t3.*t33.*t40.*t_r,0.0,-t33.*(t43+t46+C_delta.*V-t3.*t25.*t_f+a_f0.*t2.*t3.*t40),0.0,0.0,0.0,0.0,Z_f.*k_kappa_f.*t6.*t33,0.0,0.0];
mt5 = [0.0,0.0,-Z_f.*k_kappa_f.*t33.*1.0./t41.^2];
H = reshape([mt1,mt2,mt3,mt4,mt5],9,9);
end
