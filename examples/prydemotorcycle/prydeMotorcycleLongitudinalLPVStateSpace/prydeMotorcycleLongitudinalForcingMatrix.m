function H = prydeMotorcycleLongitudinalForcingMatrix(in1)
%prydeMotorcycleLongitudinalForcingMatrix
%    H = prydeMotorcycleLongitudinalForcingMatrix(IN1)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    01-Apr-2025 19:11:49

%   states = [v_rx omega_r omega_f]
%   inputs = [tau_r tau_delta]
%   params = [C_delta I_bxx I_bxz I_bzz I_hxx I_hzz K_xkf K_xkr K_yaf K_ygf K_yar K_ygr K_zaf K_zgf K_zar K_zgr V a a_n b e f g h i_fy i_ry m_b m_h rho_f rho_r t_f t_r varepsilon]
%
C_delta = in1(1,:);
K_xkf = in1(7,:);
K_xkr = in1(8,:);
K_yaf = in1(9,:);
K_ygf = in1(10,:);
K_yar = in1(11,:);
K_ygr = in1(12,:);
K_zaf = in1(13,:);
K_zgf = in1(14,:);
K_zar = in1(15,:);
K_zgr = in1(16,:);
V = in1(17,:);
a = in1(18,:);
a_n = in1(19,:);
b = in1(20,:);
e = in1(21,:);
f = in1(22,:);
g = in1(23,:);
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
t4 = K_yaf.*a;
t5 = K_yaf.*a_n;
t6 = K_yaf.*rho_r;
t7 = K_yar.*rho_r;
t8 = K_yaf.*t_f;
t9 = K_yaf.*t_r;
t10 = rho_f+t_f;
t11 = rho_r+t_r;
t12 = V.^2;
t13 = a.^2;
t14 = a_n.^2;
t15 = t_f.^2;
t16 = varepsilon.*2.0;
t17 = K_yaf+K_yar;
t26 = 1.0./V;
t28 = -a_n;
t35 = a.*e.*g.*m_h;
t36 = a_n.*e.*g.*m_h;
t18 = t2.^2;
t19 = t2.^3;
t21 = sin(t16);
t22 = t3.^2;
t23 = t3.^3;
t24 = K_zaf.*t2;
t25 = b.*t2;
t27 = t3.*t_f;
t29 = -t5;
t30 = -t8;
t31 = 1.0./t2;
t32 = a_n.*t5;
t38 = 1.0./t10;
t39 = 1.0./t11;
t45 = f.*g.*m_h.*t2.*t_f;
t46 = -t35;
t47 = b.*t6.*t26;
t48 = b.*t9.*t26;
t52 = K_xkf.*t10.*t26;
t53 = K_xkr.*t11.*t26;
t20 = t18.^2;
t34 = K_yaf.*t25;
t37 = -t24;
t40 = -t23;
t43 = e.*g.*m_h.*t25;
t44 = t24.*t28;
t51 = f.*g.*m_h.*t19.*t_f;
t54 = V.*i_fy.*t38;
t55 = V.*i_ry.*t39;
t56 = g.*m_b.*t25.*t28;
t57 = g.*m_h.*t25.*t28;
t59 = -t45;
t60 = a+t25+t28;
t62 = a.*g.*m_h.*t18.*t28;
t63 = e.*g.*m_h.*t18.*t28;
t64 = -t52;
t65 = -t53;
t66 = (a_n.*f.*g.*m_h.*t21)./2.0;
t67 = (b.*g.*m_b.*t21.*t_f)./2.0;
t68 = (b.*g.*m_h.*t21.*t_f)./2.0;
t69 = rho_r.*t4.*t26.*t31;
t71 = t4.*t26.*t31.*t_r;
t74 = rho_r.*t26.*t29.*t31;
t75 = t26.*t29.*t31.*t_r;
t76 = t6+t7+t9+t30;
t58 = -t43;
t61 = t3+t40;
t73 = 1.0./t60;
t77 = a.*g.*m_h.*t61.*t_f;
t78 = e.*g.*m_h.*t61.*t_f;
et1 = t3.*t35+K_ygf.*a.*rho_r+K_ygr.*a.*rho_r+K_ygf.*a.*t_r+K_ygr.*a.*t_r+K_ygf.*rho_r.*t25+K_ygf.*rho_r.*t28+K_ygr.*rho_r.*t25+K_ygr.*rho_r.*t28+K_ygf.*t25.*t_r+K_ygf.*t28.*t_r+K_ygr.*t25.*t_r+K_ygr.*t28.*t_r-a.*g.*m_b.*t_r-a.*g.*m_h.*t_r+a_n.*g.*m_b.*t_r+a_n.*g.*m_h.*t_r+g.*h.*m_b.*t25+g.*h.*m_b.*t28+g.*m_h.*t3.*t13-g.*m_b.*t25.*t_f-g.*m_h.*t25.*t_f+a.*g.*h.*m_b+(a.*b.*g.*m_h.*t21)./2.0+a.*f.*g.*m_h.*t2+(b.*e.*g.*m_h.*t21)./2.0+b.*f.*g.*m_h.*t18+a.*g.*m_h.*t3.*t28-a.*g.*m_h.*t18.*t_f+a.*g.*m_h.*t18.*t_r+e.*g.*m_h.*t3.*t28-e.*g.*m_h.*t18.*t_f+e.*g.*m_h.*t18.*t_r+f.*g.*m_h.*t2.*t28+(f.*g.*m_h.*t21.*t_f)./2.0;
et2 = f.*g.*m_h.*t21.*t_r.*(-1.0./2.0);
et3 = 1.0./(a.*2.0-a_n.*2.0+t25.*2.0);
et4 = t2.*t32.*2.0+t3.*t35.*2.0-t23.*t36.*2.0+K_zaf.*a.*t18.*2.0-K_zgf.*a.*t21-K_zaf.*a_n.*t18.*2.0+K_zgf.*a_n.*t21-K_zgf.*b.*t3.*2.0+K_zaf.*b.*t19.*2.0+K_zgf.*b.*t23.*2.0-K_ygf.*t3.*t14.*2.0-a_n.*t2.*t4.*2.0-b.*t5.*t18.*2.0-g.*m_b.*t25.*t_f.*2.0-g.*m_h.*t25.*t_f.*2.0+K_ygf.*a.*a_n.*t3.*2.0+K_ygf.*a_n.*b.*t21+a.*a_n.*g.*m_h.*t3.*2.0-a.*a_n.*g.*m_h.*t23.*2.0+a_n.*b.*g.*m_b.*t21+a_n.*b.*g.*m_h.*t21+b.*e.*g.*m_h.*t21-a_n.*f.*g.*m_h.*t2.*2.0+a_n.*f.*g.*m_h.*t19.*2.0-a.*g.*m_h.*t18.*t_f.*2.0+a.*g.*m_h.*t20.*t_f.*2.0+b.*g.*m_b.*t19.*t_f.*2.0+b.*g.*m_h.*t19.*t_f.*2.0-e.*g.*m_h.*t18.*t_f.*2.0+e.*g.*m_h.*t20.*t_f.*2.0-f.*g.*m_h.*t19.*t27.*2.0+f.*g.*m_h.*t21.*t_f;
et5 = -1.0;
mt1 = [0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,V,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-K_zgf-K_zgr-K_ygf.*b-K_ygf.*a.*t31+K_ygf.*a_n.*t31,0.0,-K_ygf-K_ygr,-t73.*(et1+et2),0.0,-t73.*(t36+t46+t51+t56+t57+t58+t59+t62+t63+t66+t67+t68+t77+t78+K_ygf.*t14+K_ygf.*a.*t28+K_zgf.*a.*t2+K_zgf.*b.*t18+K_ygf.*t25.*t28+K_zgf.*t2.*t28),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t4+t5-t34+t37+K_zgf.*t3+K_ygf.*b.*t3+K_ygf.*t3.*t28.*t31+K_ygf.*a.*t3.*t31,0.0,-K_yaf.*t2+K_ygf.*t3,-t73.*(t36+t46+t51+t56+t57+t58+t59+t62+t63+t66+t67+t68+t77+t78+b.*t6.*t18+b.*t9.*t18+rho_r.*t2.*t4+rho_r.*t2.*t29+t2.*t4.*t_r+t2.*t29.*t_r-K_ygf.*a.*rho_r.*t3+K_ygf.*a_n.*rho_r.*t3-(K_ygf.*b.*rho_r.*t21)./2.0-K_ygf.*a.*t3.*t_r+K_ygf.*a_n.*t3.*t_r-(K_ygf.*b.*t21.*t_r)./2.0),0.0,et3.*et4.*et5,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,t31.*t60,0.0];
mt3 = [(t26.*(t32+t44+a.*t4+a.*t24-a_n.*t4.*2.0+t4.*t25.*2.0-t5.*t25.*2.0+K_zaf.*b.*t18+K_yaf.*b.^2.*t18+a.*m_h.*t12.*t19+b.*m_b.*t12.*t18+b.*m_h.*t12.*t18+e.*m_h.*t12.*t19-f.*m_h.*t3.*t12.*t18))./t18,0.0,t26.*t31.*(t4+t29+t34+m_b.*t2.*t12+m_h.*t2.*t12),t47+t48-t54-t55+t69+t71+t74+t75-V.*h.*m_b+V.*m_b.*rho_r+V.*m_b.*t_r+V.*m_h.*rho_r.*t18+V.*m_h.*rho_r.*t22+V.*m_h.*t18.*t_r+V.*m_h.*t22.*t_r-V.*a.*m_h.*t3-V.*e.*m_h.*t3-V.*f.*m_h.*t2,0.0];
mt4 = [t26.*t31.*t38.*(rho_f.*t32+rho_f.*t44+t32.*t_f+t44.*t_f+a.*rho_f.*t24+a.*t24.*t_f+(i_fy.*t12.*t21)./2.0+rho_f.*t4.*t28+rho_f.*t25.*t29+t4.*t28.*t_f+t25.*t29.*t_f+K_zaf.*b.*rho_f.*t18+K_zaf.*b.*t18.*t_f+e.*m_h.*rho_f.*t2.*t12+e.*m_h.*t2.*t12.*t_f),0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t26.*(K_xkf+K_xkr),0.0,0.0,t65,0.0,t64,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,t26.*t31.*(t4+t24+t29+t34+K_zar.*t2),0.0,t17.*t26,t11.*t17.*t26,0.0,-t26.*(t5+t37),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,-t10+t11,1.0];
mt5 = [t47+t48+t54+t55+t69+t71+t74+t75+K_zaf.*rho_f.*t26+K_zaf.*rho_r.*t26+K_zar.*rho_r.*t26+K_zaf.*t26.*t_r-K_zaf.*rho_f.*t18.*t26-K_zaf.*rho_f.*t22.*t26-K_zaf.*t18.*t26.*t_f-K_zaf.*t22.*t26.*t_f+b.*t18.*t26.*t30+b.*t22.*t26.*t30-t4.*t26.*t31.*t_f+t5.*t26.*t31.*t_f+K_yaf.*b.*rho_f.*t26-K_yaf.*b.*rho_f.*t18.*t26-K_yaf.*b.*rho_f.*t22.*t26,0.0,t26.*t76,t11.*t26.*t76,0.0,t26.*t38.*(t5.*t15+t15.*t37+i_fy.*t2.*t12+rho_f.*rho_r.*t24+rho_f.*rho_r.*t29+rho_f.*t5.*t_f+rho_r.*t24.*t_f+rho_f.*t37.*t_f+rho_r.*t29.*t_f+rho_f.*t24.*t_r+rho_f.*t29.*t_r+t24.*t_f.*t_r+t29.*t_f.*t_r),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t65,0.0,0.0,K_xkr.*t26.*1.0./t39.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt6 = [1.0,0.0,0.0,t2,0.0,t27+t28+rho_f.*t3,-t3,-t3.*t54+K_zaf.*t26.*t27+K_zaf.*t26.*t28+b.*t26.*t29+t26.*t31.*t32+b.*t3.*t8.*t26+t4.*t26.*t27.*t31+t4.*t26.*t28.*t31+t26.*t27.*t29.*t31,0.0,-K_yaf.*t26.*(a_n-t27),-t2.*t54+rho_r.*t26.*t29+t6.*t26.*t27+t26.*t29.*t_r+t3.*t8.*t26.*t_r,0.0,t26.*(t32+t44+t24.*t27+t27.*t29+C_delta.*V),0.0,0.0,0.0,0.0,-V.*t2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t64,0.0,0.0,0.0,0.0,K_xkf.*t26.*1.0./t38.^2,0.0,0.0,0.0,0.0,0.0];
H = reshape([mt1,mt2,mt3,mt4,mt5,mt6],24,14);
end
