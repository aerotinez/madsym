function A = sharpMotorcycleStateMatrix(in1,in2)
%sharpMotorcycleStateMatrix
%    A = sharpMotorcycleStateMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    19-Sep-2024 17:40:41

%   states = [varphi delta omega_psi omega_varphi omega_delta v_y Y_r Y_f]
%   inputs = [tau_delta]
%   params = [C_delta C_f1 C_f2 C_r1 C_r2 C_rxz I_fx I_fz I_rx I_rz Z_f a a_n b e g h i_fy i_ry j k m_f m_r r_f r_r sigma v_x varepsilon]
%
C_delta = in2(1,:);
C_f1 = in2(2,:);
C_f2 = in2(3,:);
C_r1 = in2(4,:);
C_r2 = in2(5,:);
C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_rz = in2(10,:);
Z_f = in2(11,:);
a = in2(12,:);
a_n = in2(13,:);
b = in2(14,:);
e = in2(15,:);
g = in2(16,:);
h = in2(17,:);
i_fy = in2(18,:);
i_ry = in2(19,:);
j = in2(20,:);
k = in2(21,:);
m_f = in2(22,:);
m_r = in2(23,:);
r_f = in2(24,:);
r_r = in2(25,:);
sigma = in2(26,:);
v_x = in2(27,:);
varepsilon = in2(28,:);
t2 = cos(varepsilon);
t3 = sin(varepsilon);
t4 = Z_f.*a_n;
t5 = h.*m_r;
t6 = j.*m_f;
t7 = m_f+m_r;
t8 = e.^2;
t9 = h.^2;
t10 = j.^2;
t11 = k.^2;
t12 = m_f.^2;
t13 = varepsilon.*2.0;
t19 = e.*g.*m_f;
t22 = e.*k.*m_f;
t25 = e.*m_f.*v_x;
t27 = 1.0./r_f;
t28 = 1.0./r_r;
t29 = 1.0./sigma;
t14 = t2.^2;
t15 = sin(t13);
t16 = t3.^2;
t17 = I_fz.*t2;
t18 = I_fz.*t3;
t20 = a.*t2;
t21 = e.*t6;
t23 = e.*t2;
t24 = k.*t6;
t26 = t5.*v_x;
t30 = a.*m_f.*t3;
t31 = e.*m_f.*t3;
t32 = m_f.*t8;
t33 = h.*t5;
t34 = j.*t6;
t35 = m_f.*t11;
t37 = j.*t2.*t3;
t39 = t3.*t25;
t40 = -t19;
t41 = t29.*v_x;
t44 = i_fy.*t27.*v_x;
t45 = i_ry.*t28.*v_x;
t48 = t5+t6;
t49 = k.*m_f.*t2.*t3;
t36 = k.*t16;
t38 = t30.*v_x;
t42 = I_fx.*t16;
t43 = I_fz.*t16;
t46 = I_fz+t32;
t47 = t6.*t14;
t50 = t49.*v_x;
t51 = -t37;
t52 = -t41;
t56 = t48.^2;
t57 = t17+t22;
t58 = t18+t21;
t59 = (I_fx.*t15)./2.0;
t60 = (I_fz.*t15)./2.0;
t61 = t3.*t44;
t62 = -t49;
t63 = t4+t40;
t72 = t44+t45;
t53 = t47.*v_x;
t54 = -t42;
t55 = -t43;
t64 = -t50;
t65 = -t59;
t66 = t57.^2;
t67 = t58.^2;
t68 = t25+t61;
t70 = t11.*t12.*t46;
t74 = e.*k.*t12.*t58;
t75 = e.*k.*t12.*t57.*2.0;
t76 = t11.*t12.*t58;
t77 = k.*m_f.*t46.*t48;
t80 = t46.*t56;
t82 = t56.*t57;
t84 = e.*m_f.*t48.*t57;
t85 = k.*m_f.*t48.*t57;
t86 = k.*m_f.*t48.*t58;
t87 = e.*m_f.*t48.*t58.*2.0;
t89 = k.*m_f.*t57.*t58;
t96 = t7.*t57.*t58;
t97 = t20+t23+t36+t51;
t101 = t48.*t57.*t58;
t107 = t5+t30+t31+t47+t62;
t69 = k.*m_f.*t67;
t71 = t7.*t66;
t73 = t7.*t67;
t78 = -t75;
t79 = -t74;
t81 = t11.*t12.*t67;
t83 = t48.*t66;
t90 = t56.*t66;
t91 = -t84;
t92 = -t85;
t93 = -t87;
t94 = -t86;
t95 = I_fz+I_rz+t35+t42+t55;
t98 = -t89;
t100 = C_rxz+t24+t60+t65;
t102 = I_fx+I_rx+t33+t34+t43+t54;
t104 = -t101;
t105 = t58.*t85.*2.0;
t168 = t26+t38+t39+t53+t64+t72;
t88 = -t81;
t99 = -t90;
t103 = t100.^2;
t106 = t8.*t12.*t95;
t110 = e.*k.*t12.*t100;
t111 = t8.*t12.*t100;
t112 = e.*m_f.*t48.*t95;
t113 = t56.*t95;
t115 = t7.*t46.*t95;
t116 = e.*k.*t12.*t102;
t117 = e.*m_f.*t58.*t95;
t119 = t8.*t12.*t102;
t120 = t11.*t12.*t102;
t121 = e.*m_f.*t48.*t100;
t123 = t7.*t58.*t95;
t124 = k.*m_f.*t46.*t100;
t125 = k.*m_f.*t48.*t100.*2.0;
t127 = t7.*t46.*t100;
t130 = k.*m_f.*t46.*t102;
t132 = t46.*t48.*t95;
t133 = e.*m_f.*t57.*t100;
t134 = e.*m_f.*t58.*t100;
t135 = k.*m_f.*t58.*t100;
t136 = t73.*t95;
t137 = t7.*t57.*t100;
t138 = t7.*t58.*t100;
t140 = t7.*t46.*t102;
t142 = t48.*t58.*t95;
t143 = t80.*t95;
t145 = e.*m_f.*t57.*t102;
t146 = k.*m_f.*t57.*t102;
t151 = t7.*t57.*t102;
t152 = t46.*t48.*t100;
t154 = t70.*t102;
t155 = t74.*t100.*2.0;
t157 = t71.*t102;
t158 = t48.*t57.*t100;
t159 = t87.*t95;
t160 = t77.*t100.*2.0;
t161 = t75.*t102;
t166 = t84.*t100.*2.0;
t167 = t96.*t100.*2.0;
t170 = e.*m_f.*t95.*t102;
t171 = t7.*t95.*t102;
t108 = e.*m_f.*t103;
t109 = t7.*t103;
t114 = -t110;
t118 = t8.*t12.*t103;
t122 = -t115;
t128 = -t121;
t129 = -t125;
t131 = -t123;
t141 = -t127;
t144 = -t130;
t147 = -t132;
t148 = -t133;
t149 = -t134;
t150 = -t135;
t153 = -t140;
t156 = -t151;
t162 = t58.*t112.*-2.0;
t163 = -t160;
t164 = t57.*t116.*-2.0;
t165 = -t158;
t169 = -t167;
t172 = -t170;
t173 = -t171;
t174 = t102.*t106;
t175 = t102.*t115;
t126 = -t118;
t139 = t46.*t109;
t176 = t102.*t122;
t177 = t70+t71+t78+t106+t122;
t178 = t73+t80+t93+t119+t153;
t179 = t77+t79+t91+t96+t111+t141;
t180 = t76+t92+t112+t114+t131+t137;
t181 = t83+t98+t117+t124+t147+t148;
t182 = t109+t113+t120+t129+t173;
t183 = t82+t94+t116+t128+t138+t156;
t184 = t69+t104+t144+t145+t149+t152;
t185 = t108+t142+t146+t150+t165+t172;
t186 = t88+t99+t105+t126+t136+t139+t143+t154+t155+t157+t162+t163+t164+t166+t169+t174+t176;
t187 = 1.0./t186;
t188 = t2.*t44.*t180.*t187;
t189 = t63.*t180.*t187;
mt1 = [0.0,0.0,t63.*t183.*t187+g.*t107.*t187.*(t74-t77+t84-t96-t111+t127),t189+g.*t107.*t177.*t187,-t63.*t182.*t187-g.*t107.*t180.*t187,t63.*t185.*t187-g.*t107.*t181.*t187,C_r2.*t41,C_f2.*t41,0.0,0.0,-t63.*t187.*(t74-t77+t84-t96-t111+t127)+t3.*t63.*t183.*t187,t3.*t189-t63.*t177.*t187,t189-t3.*t63.*t182.*t187,t63.*t181.*t187+t3.*t63.*t185.*t187,0.0,t41.*(C_f1.*t2+C_f2.*t3),0.0,0.0,-t168.*t187.*(t74-t77+t84-t96-t111+t127)+t68.*t183.*t187+t7.*t184.*t187.*v_x-m_f.*t97.*t178.*t187.*v_x];
mt2 = [t68.*t180.*t187-t168.*t177.*t187+t7.*t181.*t187.*v_x-m_f.*t97.*t187.*v_x.*(t74-t77+t84-t96-t111+t127),-t68.*t182.*t187+t168.*t180.*t187+t7.*t185.*t187.*v_x+m_f.*t97.*t183.*t187.*v_x,t68.*t185.*t187+t168.*t181.*t187-t7.*t187.*v_x.*(t46.*t103+t67.*t95+t66.*t102-t57.*t58.*t100.*2.0-t46.*t95.*t102)+m_f.*t97.*t184.*t187.*v_x,C_r1.*b.*t29,-(C_f1.*t29.*(a-a_n))./t2,1.0,0.0,t72.*t178.*t187-t2.*t44.*t183.*t187,-t188+t72.*t187.*(t74-t77+t84-t96-t111+t127),-t72.*t183.*t187+t2.*t44.*t182.*t187,-t72.*t184.*t187-t2.*t44.*t185.*t187,0.0,0.0,0.0,1.0];
mt3 = [C_delta.*t183.*t187+t61.*t178.*t187-t2.*t44.*t187.*(t74-t77+t84-t96-t111+t127),t61.*t187.*(t74-t77+t84-t96-t111+t127)+C_delta.*t180.*t187-t2.*t44.*t177.*t187,t188-C_delta.*t182.*t187-t61.*t183.*t187,C_delta.*t185.*t187-t61.*t184.*t187+t2.*t44.*t181.*t187,0.0,C_f1.*a_n.*t29,0.0,0.0,0.0,0.0,0.0,0.0,-C_r1.*t29,-C_f1.*t29,0.0,0.0,0.0,0.0,0.0,0.0,t52,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t52];
A = reshape([mt1,mt2,mt3],8,8);
