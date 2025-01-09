function H = prydeMotorcycleForcingMatrix(in1)
%prydeMotorcycleForcingMatrix
%    H = prydeMotorcycleForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    09-Jan-2025 17:19:44

%   states = [varphi delta omega_bz v_rx v_ry omega_bx omega_r omega_delta omega_f]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz Psi_alpha_f Psi_alpha_r Psi_varphi_f Psi_varphi_r V Y_alpha_f Y_alpha_r Y_varphi_f Y_varphi_r Z_f Z_r a a_n b e f g h i_fy i_ry k_kappa_f k_kappa_r m_b m_h rho_f rho_r t_f t_r varepsilon]
%
C_delta = in1(1,:);
Psi_alpha_f = in1(7,:);
Psi_alpha_r = in1(8,:);
Psi_varphi_f = in1(9,:);
Psi_varphi_r = in1(10,:);
V = in1(11,:);
Y_alpha_f = in1(12,:);
Y_alpha_r = in1(13,:);
Y_varphi_f = in1(14,:);
Y_varphi_r = in1(15,:);
Z_f = in1(16,:);
Z_r = in1(17,:);
a = in1(18,:);
a_n = in1(19,:);
b = in1(20,:);
e = in1(21,:);
f = in1(22,:);
g = in1(23,:);
h = in1(24,:);
i_fy = in1(25,:);
i_ry = in1(26,:);
k_kappa_f = in1(27,:);
k_kappa_r = in1(28,:);
m_b = in1(29,:);
m_h = in1(30,:);
rho_f = in1(31,:);
rho_r = in1(32,:);
t_f = in1(33,:);
t_r = in1(34,:);
varepsilon = in1(35,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = Y_alpha_f.*Z_f;
t5 = Y_alpha_r.*Z_r;
t6 = rho_f+t_f;
t7 = rho_r+t_r;
t8 = V.^2;
t9 = a.^2;
t10 = a_n.^2;
t11 = t_f.^2;
t12 = varepsilon.*2.0;
t21 = Z_f.*a.*a_n;
t26 = Z_f.*a.*t_f;
t27 = Z_f.*a_n.*t_f;
t29 = 1.0./V;
t31 = -a_n;
t34 = a.*e.*g.*m_h;
t35 = a_n.*e.*g.*m_h;
t13 = t2.^2;
t14 = t2.^3;
t16 = sin(t12);
t17 = t3.^2;
t18 = t3.^3;
t19 = a.*t4;
t20 = a_n.*t4;
t22 = rho_r.*t4;
t23 = rho_r.*t5;
t24 = t4.*t_f;
t25 = t4.*t_r;
t28 = b.*t2;
t30 = t3.*t_f;
t32 = 1.0./t2;
t33 = Z_f.*t10;
t41 = t3.*t26;
t42 = t3.*t27;
t43 = -t21;
t45 = -t27;
t46 = 1.0./t6;
t47 = 1.0./t7;
t48 = t4.*t10;
t49 = t4+t5;
t52 = t3.*t34;
t53 = t3.*t35;
t54 = e.*g.*m_h.*t31;
t55 = Psi_alpha_f.*Z_f.*t2.*t31;
t62 = f.*g.*m_h.*t2.*t_f.*2.0;
t75 = Z_f.*k_kappa_f.*t6.*t29;
t76 = Z_r.*k_kappa_r.*t7.*t29;
t15 = t13.^2;
t37 = t4.*t28;
t38 = Z_f.*a_n.*t28;
t39 = Z_f.*t28.*t_f;
t40 = -t20;
t44 = -t24;
t50 = -t18;
t51 = e.*g.*m_h.*t28;
t56 = Z_f.*t28.*t31;
t58 = a_n.*g.*m_b.*t28.*2.0;
t59 = a_n.*g.*m_h.*t28.*2.0;
t60 = g.*m_b.*t28.*t_f.*2.0;
t61 = g.*m_h.*t28.*t_f.*2.0;
t63 = a_n.*f.*g.*m_h.*t16;
t64 = b.*g.*m_b.*t16.*t_f;
t65 = b.*g.*m_h.*t16.*t_f;
t66 = f.*g.*m_h.*t16.*t_f;
t67 = b.*t20.*t29;
t68 = b.*t22.*t29;
t69 = b.*t25.*t29;
t70 = V.*i_fy.*t46;
t71 = V.*i_ry.*t47;
t74 = a+t28+t31;
t77 = a.*a_n.*g.*m_h.*t13.*2.0;
t78 = t13.*t35.*2.0;
t79 = a.*g.*m_h.*t13.*t_f.*2.0;
t80 = e.*g.*m_h.*t13.*t_f.*2.0;
t81 = f.*g.*m_h.*t14.*t_f.*2.0;
t92 = (Z_f.*b.*t16.*t_f)./2.0;
t93 = (b.*e.*g.*m_h.*t16)./2.0;
t95 = a_n.*t19.*t29.*t32;
t96 = rho_r.*t19.*t29.*t32;
t97 = rho_r.*t20.*t29.*t32;
t98 = t19.*t29.*t32.*t_r;
t99 = t20.*t29.*t32.*t_r;
t100 = t29.*t32.*t48;
t72 = -t60;
t73 = -t61;
t82 = t3+t50;
t83 = -t79;
t85 = -t80;
t89 = -t68;
t90 = -t69;
t91 = t3.*t70;
t94 = 1.0./t74;
t101 = -t96;
t102 = -t98;
t105 = -t100;
t108 = t22+t23+t25+t44;
t103 = a.*g.*m_h.*t82.*t_f.*2.0;
t104 = e.*g.*m_h.*t82.*t_f.*2.0;
mt1 = [0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Psi_varphi_f.*Z_f+Psi_varphi_r.*Z_r+Y_varphi_f.*Z_f.*b+Y_varphi_f.*Z_f.*a.*t32+Y_varphi_f.*Z_f.*t31.*t32,0.0,Y_varphi_f.*Z_f+Y_varphi_r.*Z_r];
mt2 = [t94.*(t26+t39+t45+t52+t66+t72+t73+t83+t85+t93+t3.*t54-Z_f.*a.*t_r+Z_f.*a_n.*t_r-Z_f.*t28.*t_r-a.*g.*m_b.*t_r-a.*g.*m_h.*t_r+a_n.*g.*m_b.*t_r+a_n.*g.*m_h.*t_r+g.*h.*m_b.*t28+g.*h.*m_b.*t31+g.*m_h.*t3.*t9+g.*m_b.*t28.*t_r+g.*m_h.*t28.*t_r+Y_varphi_f.*Z_f.*a.*rho_r+Y_varphi_r.*Z_r.*a.*rho_r+Y_varphi_f.*Z_f.*a.*t_r+Y_varphi_r.*Z_r.*a.*t_r+Y_varphi_f.*Z_f.*rho_r.*t28+Y_varphi_f.*Z_f.*rho_r.*t31+Y_varphi_r.*Z_r.*rho_r.*t28+Y_varphi_r.*Z_r.*rho_r.*t31+Y_varphi_f.*Z_f.*t28.*t_r+Y_varphi_f.*Z_f.*t31.*t_r+Y_varphi_r.*Z_r.*t28.*t_r+Y_varphi_r.*Z_r.*t31.*t_r+a.*g.*h.*m_b+(a.*b.*g.*m_h.*t16)./2.0+a.*f.*g.*m_h.*t2+b.*f.*g.*m_h.*t13+a.*g.*m_h.*t3.*t31+a.*g.*m_h.*t13.*t_r.*2.0+e.*g.*m_h.*t13.*t_r.*2.0+f.*g.*m_h.*t2.*t31-f.*g.*m_h.*t16.*t_r)];
mt3 = [0.0,t94.*(t21-t33-t34+t35+t38-t41+t42-t51-t58-t59-t62+t63+t64+t65-t77-t78+t81-t92+t103+t104+Y_varphi_f.*t33+Y_varphi_f.*t43+Y_varphi_f.*t56+Psi_varphi_f.*Z_f.*a.*t2+Psi_varphi_f.*Z_f.*b.*t13+Psi_varphi_f.*Z_f.*t2.*t31),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Z_f.*t32.*(Psi_alpha_f.*t13-(Psi_varphi_f.*t16)./2.0+Y_alpha_f.*a.*t2-Y_varphi_f.*a.*t3+Y_varphi_f.*a_n.*t3+Y_alpha_f.*b.*t13-(Y_varphi_f.*b.*t16)./2.0+Y_alpha_f.*t2.*t31),0.0,Z_f.*(Y_alpha_f.*t2-Y_varphi_f.*t3)];
mt4 = [t94.*(t21-t33-t34+t35+t38-t41+t42-t51-t58-t59-t62+t63+t64+t65-t77-t78+t81-t92+t103+t104+b.*t13.*t22+b.*t13.*t25+rho_r.*t2.*t19+rho_r.*t2.*t40+t2.*t19.*t_r+t2.*t40.*t_r-Y_varphi_f.*Z_f.*a.*rho_r.*t3+Y_varphi_f.*Z_f.*a_n.*rho_r.*t3-(Y_varphi_f.*Z_f.*b.*rho_r.*t16)./2.0-Y_varphi_f.*Z_f.*a.*t3.*t_r+Y_varphi_f.*Z_f.*a_n.*t3.*t_r-(Y_varphi_f.*Z_f.*b.*t16.*t_r)./2.0),0.0];
mt5 = [t94.*(t26+t39+t45+t52+t53+t66+t72+t73+t83+t85+t93+t3.*t33-t13.*t26+t13.*t27+t3.*t43+t2.*t48-t18.*t35.*2.0+Y_varphi_f.*t3.*t21-Y_varphi_f.*t3.*t33+b.*t13.*t40+t2.*t19.*t31-Z_f.*b.*t14.*t_f+Psi_alpha_f.*Z_f.*a.*t13-(Psi_varphi_f.*Z_f.*a.*t16)./2.0+(Psi_varphi_f.*Z_f.*a_n.*t16)./2.0+Psi_alpha_f.*Z_f.*b.*t14-Psi_varphi_f.*Z_f.*b.*t3+Psi_varphi_f.*Z_f.*b.*t18-(Z_f.*a_n.*b.*t16)./2.0+Psi_alpha_f.*Z_f.*t13.*t31+(Y_varphi_f.*Z_f.*a_n.*b.*t16)./2.0+a.*a_n.*g.*m_h.*t3.*2.0-a.*a_n.*g.*m_h.*t18.*2.0+a_n.*b.*g.*m_b.*t16+a_n.*b.*g.*m_h.*t16-a_n.*f.*g.*m_h.*t2.*2.0+a_n.*f.*g.*m_h.*t14.*2.0+a.*g.*m_h.*t15.*t_f.*2.0+b.*g.*m_b.*t14.*t_f.*2.0+b.*g.*m_h.*t14.*t_f.*2.0+e.*g.*m_h.*t15.*t_f.*2.0-f.*g.*m_h.*t14.*t30.*2.0)];
mt6 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,t32.*t74,0.0,-(t29.*(t48+t55-a_n.*t19.*2.0+t4.*t9+t19.*t28.*2.0-t20.*t28.*2.0+b.^2.*t4.*t13+a.*m_h.*t8.*t14+b.*m_b.*t8.*t13+b.*m_h.*t8.*t13+e.*m_h.*t8.*t14+Psi_alpha_f.*Z_f.*a.*t2+Psi_alpha_f.*Z_f.*b.*t13-f.*m_h.*t3.*t8.*t13))./t13,0.0,-t29.*t32.*(t19+t37+t40+m_b.*t2.*t8+m_h.*t2.*t8),t70+t71+t89+t90+t97+t99+t101+t102+V.*h.*m_b-V.*m_b.*rho_r-V.*m_h.*rho_r-V.*m_b.*t_r-V.*m_h.*t_r+V.*a.*m_h.*t3+V.*e.*m_h.*t3+V.*f.*m_h.*t2,0.0];
mt7 = [t67-t91+t95+t105-V.*e.*m_h-Psi_alpha_f.*Z_f.*t28.*t29-Psi_alpha_f.*Z_f.*a.*t13.*t29-Psi_alpha_f.*Z_f.*a.*t17.*t29+Psi_alpha_f.*Z_f.*a_n.*t13.*t29+Psi_alpha_f.*Z_f.*a_n.*t17.*t29,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-t29.*(Z_f.*k_kappa_f+Z_r.*k_kappa_r),0.0,0.0,t76,0.0,t75,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,-t29.*t32.*(t19+t37+t40+Psi_alpha_f.*Z_f.*t2+Psi_alpha_r.*Z_r.*t2),0.0,-t29.*t49,-t7.*t29.*t49,0.0,Z_f.*t29.*(Y_alpha_f.*a_n-Psi_alpha_f.*t2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-t6+t7,1.0];
mt8 = [-t70-t71+t89+t90+t97+t99+t101+t102-b.*rho_f.*t4.*t29+b.*t13.*t24.*t29+b.*t17.*t24.*t29+t19.*t29.*t32.*t_f+t29.*t32.*t40.*t_f-Psi_alpha_f.*Z_f.*rho_f.*t29-Psi_alpha_f.*Z_f.*rho_r.*t29-Psi_alpha_r.*Z_r.*rho_r.*t29-Psi_alpha_f.*Z_f.*t29.*t_r+Psi_alpha_f.*Z_f.*rho_f.*t13.*t29+Psi_alpha_f.*Z_f.*rho_f.*t17.*t29+Psi_alpha_f.*Z_f.*t13.*t29.*t_f+Psi_alpha_f.*Z_f.*t17.*t29.*t_f+b.*rho_f.*t4.*t13.*t29+b.*rho_f.*t4.*t17.*t29,0.0,-t29.*t108,-t7.*t29.*t108,0.0,-t29.*t46.*(t11.*t20+i_fy.*t2.*t8+rho_f.*rho_r.*t40+rho_f.*t20.*t_f+rho_r.*t40.*t_f+rho_f.*t40.*t_r+t40.*t_f.*t_r-Psi_alpha_f.*Z_f.*t2.*t11+Psi_alpha_f.*Z_f.*rho_f.*rho_r.*t2-Psi_alpha_f.*Z_f.*rho_f.*t2.*t_f+Psi_alpha_f.*Z_f.*rho_r.*t2.*t_f+Psi_alpha_f.*Z_f.*rho_f.*t2.*t_r+Psi_alpha_f.*Z_f.*t2.*t_f.*t_r),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt9 = [0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t76,0.0,0.0,-Z_r.*k_kappa_r.*t29.*1.0./t47.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,t2,0.0,t30+t31+rho_f.*t3,-t3,t67+t91+t95+t105+b.*t3.*t29.*t44-t19.*t29.*t30.*t32+t20.*t29.*t30.*t32+Psi_alpha_f.*Z_f.*a_n.*t29-Psi_alpha_f.*Z_f.*t29.*t30,0.0,t4.*t29.*(a_n-t30),t2.*t70+rho_r.*t20.*t29-t22.*t29.*t30+t20.*t29.*t_r+t3.*t29.*t44.*t_r,0.0,-t29.*(t48+t55+t30.*t40+C_delta.*V+Psi_alpha_f.*Z_f.*t2.*t30),0.0,0.0,0.0,0.0,-V.*t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t75,0.0,0.0,0.0,0.0,-Z_f.*k_kappa_f.*t29.*1.0./t46.^2,0.0,0.0,0.0,0.0,0.0];
H = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8,mt9],24,14);
end
