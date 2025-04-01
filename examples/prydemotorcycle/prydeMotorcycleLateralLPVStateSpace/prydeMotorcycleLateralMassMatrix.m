function M = prydeMotorcycleLateralMassMatrix(in1)
%prydeMotorcycleLateralMassMatrix
%    M = prydeMotorcycleLateralMassMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    01-Apr-2025 13:01:05

%   states = [varphi delta omega_bz v_ry omega_bx omega_delta]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
%
I_bxx = in1(2,:);
I_bxz = in1(3,:);
I_bzz = in1(4,:);
I_hxx = in1(5,:);
I_hzz = in1(6,:);
a = in1(18,:);
a_n = in1(19,:);
b = in1(20,:);
e = in1(21,:);
f = in1(22,:);
h = in1(24,:);
i_fy = in1(25,:);
i_ry = in1(26,:);
m_b = in1(27,:);
m_h = in1(28,:);
rho_f = in1(29,:);
rho_r = in1(30,:);
t_f = in1(31,:);
t_r = in1(32,:);
varepsilon = in1(33,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = b.*m_b;
t5 = b.*m_h;
t6 = e.*m_h;
t7 = h.*m_b;
t8 = m_b.*rho_r;
t9 = m_h.*rho_r;
t10 = m_b.*t_r;
t11 = m_h.*t_r;
t12 = a.^2;
t15 = f.^2;
t18 = varepsilon.*2.0;
t34 = -I_bxz;
t35 = -a_n;
t36 = -m_b;
t37 = -m_h;
t19 = cos(t18);
t20 = t2.^2;
t21 = sin(t18);
t22 = t3.^2;
t23 = I_hzz.*t2;
t24 = I_hzz.*t3;
t25 = e.*t5;
t26 = h.*t4;
t27 = b.*t2;
t28 = rho_r.*t4;
t29 = rho_r.*t5;
t30 = t4.*t_r;
t31 = t5.*t_r;
t32 = rho_r.*t6;
t33 = t6.*t_r;
t39 = t2.*t6;
t40 = f.*m_h.*t2;
t41 = a.*m_h.*t3;
t42 = t3.*t6;
t43 = f.*m_h.*t3;
t44 = -t4;
t45 = -t5;
t46 = -t6;
t47 = 1.0./t2;
t48 = -t8;
t49 = -t9;
t50 = -t10;
t51 = -t11;
t53 = f.*t2.*t5;
t55 = a.*t3.*t5;
t64 = f.*t3.*t9;
t65 = f.*t3.*t11;
t73 = a.*t2.*t37;
t90 = t36+t37;
t52 = a.*t39;
t54 = f.*t39;
t58 = a.*t42;
t59 = t3.*t25;
t60 = t2.*t32;
t61 = t2.*t33;
t62 = f.*t42;
t63 = -t23;
t66 = -t25;
t67 = -t28;
t68 = -t29;
t69 = -t30;
t70 = -t31;
t71 = -t32;
t72 = -t33;
t74 = -t39;
t76 = e.*t42;
t78 = a.*t2.*t49;
t79 = a.*t2.*t51;
t82 = a.*f.*m_h.*t19;
t83 = f.*t6.*t19;
t84 = a.*t6.*t21;
t85 = a.*f.*m_h.*t21;
t86 = f.*t6.*t21;
t88 = (I_hxx.*t21)./2.0;
t89 = (I_hzz.*t21)./2.0;
t91 = a+t27+t35;
t93 = (m_h.*t12.*t21)./2.0;
t94 = (e.*t6.*t21)./2.0;
t95 = (m_h.*t15.*t21)./2.0;
t101 = t7+t40+t41+t42+t48+t49+t50+t51;
t77 = -t52;
t80 = -t60;
t81 = -t61;
t87 = e.*t74;
t92 = -t88;
t96 = -t95;
t97 = t47.*t91;
t98 = t43+t44+t45+t73+t74;
t99 = t24+t54+t58+t71+t72+t76;
t100 = t62+t63+t66+t77+t87;
t102 = t26+t34+t53+t55+t59+t64+t65+t67+t68+t69+t70+t78+t79+t80+t81+t82+t83+t84+t89+t92+t93+t94+t96;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-I_bzz+t85+t86-I_hxx.*t22-I_hzz.*t20+b.*t44+b.*t45-t2.*t25.*2.0-a.*t2.*t5.*2.0-a.*t6.*t20.*2.0+e.*t20.*t46+f.*t3.*t5.*2.0+t12.*t20.*t37+t15.*t22.*t37,0.0,t98,t102,0.0,t100,0.0,1.0,0.0,0.0,-t97,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t90,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt3 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t98,0.0,t90,t101,0.0,t46,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t102,0.0,t101,-I_bxx-I_hxx.*t20-I_hzz.*t22-h.*t7+rho_r.*t7.*2.0+rho_r.*t48+rho_r.*t49+t3.*t32.*2.0+t3.*t33.*2.0+t7.*t_r.*2.0-t8.*t_r.*2.0-t9.*t_r.*2.0+t50.*t_r+t51.*t_r+a.*t3.*t9.*2.0+a.*t3.*t11.*2.0-a.*t6.*t22.*2.0+e.*t22.*t46+f.*t2.*t9.*2.0+f.*t2.*t11.*2.0+f.*t21.*t46+t12.*t22.*t37+t15.*t20.*t37+a.*f.*t21.*t37,0.0,t99,0.0,0.0,-1.0,0.0,rho_f-rho_r+t_f-t_r,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-i_ry,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t100];
mt4 = [0.0,t46,t99,0.0,-I_hzz+e.*t46,0.0,t2,t3,0.0,a_n-rho_f.*t3-t3.*t_f,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-i_fy,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t7+t8+t9+t10+t11-t42+a.*t3.*t37+f.*t2.*t37,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-rho_f+rho_r-t_f+t_r,0.0,t97,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt5 = [0.0,0.0,0.0,1.0,0.0,0.0,0.0];
M = reshape([mt1,mt2,mt3,mt4,mt5],24,24);
end
