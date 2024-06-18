function Msharp = extendedSharpMotorcycleLinearizedMassMatrix(in1,in2)
%extendedSharpMotorcycleLinearizedMassMatrix
%    Msharp = extendedSharpMotorcycleLinearizedMassMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    18-Jun-2024 15:09:18

C_rxz = in2(6,:);
I_fx = in2(7,:);
I_fz = in2(8,:);
I_rx = in2(9,:);
I_ry = in2(10,:);
I_rz = in2(11,:);
a_n = in2(14,:);
delta_0 = in1(2,:);
e = in2(16,:);
h = in2(18,:);
i_fy = in2(19,:);
i_ry = in2(20,:);
j = in2(21,:);
k = in2(22,:);
m_f = in2(23,:);
m_r = in2(24,:);
psi_f0 = in1(3,:);
r_f = in2(25,:);
r_r = in2(26,:);
sigma = in2(27,:);
varphi_0 = in1(1,:);
varphi_f0 = in1(4,:);
varepsilon = in2(28,:);
t2 = cos(delta_0);
t3 = cos(psi_f0);
t4 = sin(delta_0);
t5 = cos(varphi_0);
t6 = cos(varphi_f0);
t7 = cos(varepsilon);
t8 = sin(psi_f0);
t9 = sin(varphi_0);
t10 = sin(varphi_f0);
t11 = sin(varepsilon);
t12 = e.^2;
t13 = h.^2;
t14 = r_r.^2;
t33 = 1.0./r_f;
t15 = t2.^2;
t16 = t4.^2;
t17 = t5.^2;
t18 = t6.^2;
t19 = t7.^2;
t20 = t7.^3;
t21 = t9.^2;
t22 = t10.^2;
t23 = t11.^2;
t24 = C_rxz.*t5;
t25 = I_fz.*t11;
t26 = j.*t7;
t27 = k.*t7;
t28 = a_n.*t8;
t29 = e.*t11;
t30 = i_ry.*t9;
t31 = j.*t11;
t32 = k.*t11;
t34 = t33.^2;
t35 = t2.*t5;
t36 = h.*m_r.*t5;
t37 = t4.*t5;
t38 = t4.*t9;
t39 = 1.0./t3;
t40 = 1.0./t6;
t42 = 1.0./t7;
t44 = I_fz.*t5.*t7;
t46 = e.*t2.*t9;
t47 = e.*t5.*t7;
t49 = a_n.*t5.*t11;
t50 = h.*m_r.*r_r.*t9;
t56 = t2.*t6.*t7;
t58 = t2.*t9.*t11;
t61 = a_n.*t3.*t4.*t7;
t63 = e.*m_f.*r_r.*t4.*t7;
t73 = m_f.*r_r.*t7.*t9.*t11;
t75 = r_f.*t2.*t3.*t7.*t9;
t79 = r_f.*t3.*t4.*t7.*t11;
t80 = a_n.*t3.*t7.*t9.*t11;
t41 = 1.0./t18;
t43 = e.*t23;
t45 = r_f.*t35;
t48 = a_n.*t38;
t51 = -t27;
t52 = -t31;
t53 = -t32;
t54 = e.*t19;
t55 = t11.*t38;
t57 = t11.*t35;
t59 = t5.*t7.*t25;
t60 = t28.*t35;
t64 = t29.*t37;
t65 = r_f.*t8.*t38;
t67 = t19.*t28;
t68 = r_f.*t5.*t19;
t70 = -t56;
t71 = t19+t23;
t74 = a_n.*t3.*t7.*t37;
t76 = r_f.*t10.*t56;
t77 = m_f.*r_r.*t4.*t7.*t29;
t88 = r_f.*t3.*t9.*t20;
t89 = r_f.*t8.*t11.*t19;
t93 = i_fy.*r_r.*t3.*t8.*t34;
t94 = a_n.*t3.*t7.*t58;
t95 = r_f.*t3.*t7.*t11.*t37;
t98 = -t79;
t105 = t37+t58;
t62 = a_n.*t57;
t66 = r_f.*t55;
t69 = -t55;
t72 = -t57;
t78 = t8.*t11.*t45;
t81 = -t60;
t83 = t28.*t55;
t84 = -t65;
t86 = t2.*t67;
t87 = t19.*t45;
t90 = -t67;
t91 = m_f.*t38.*t54;
t92 = t26+t53;
t96 = -t74;
t97 = -t77;
t99 = 1.0./t71;
t100 = t2.*t88;
t101 = t2.*t89;
t103 = t19.*t65;
t106 = -t93;
t107 = -t94;
t114 = m_f.*r_r.*t2.*t7.*t105;
t122 = t43+t51+t52+t54;
t82 = -t62;
t85 = -t66;
t102 = -t87;
t104 = -t91;
t108 = -t100;
t109 = -t101;
t110 = t35+t69;
t111 = t38+t72;
t117 = -t114;
t123 = t2.*t9.*t92.*t99;
t124 = t4.*t7.*t92.*t99;
t125 = t38.*t92.*t99;
t127 = t2.*t7.*t92.*t99;
t129 = t57.*t92.*t99;
t130 = t11.*t37.*t92.*t99;
t132 = t72.*t92.*t99;
t134 = -t8.*t99.*(t27+t31-t43-t54);
t135 = -t9.*t99.*(t27+t31-t43-t54);
t136 = -t2.*t11.*t99.*(t27+t31-t43-t54);
t137 = -t4.*t11.*t99.*(t27+t31-t43-t54);
t138 = t9.*t99.*(t27+t31-t43-t54);
t140 = t2.*t11.*t99.*(t27+t31-t43-t54);
t141 = t4.*t11.*t99.*(t27+t31-t43-t54);
t142 = -t7.*t35.*t99.*(t27+t31-t43-t54);
t143 = -t7.*t37.*t99.*(t27+t31-t43-t54);
t144 = t7.*t35.*t99.*(t27+t31-t43-t54);
t154 = t61+t78+t81+t83+t84+t98+t103;
t112 = e.*m_f.*t110;
t113 = r_f.*t10.*t110;
t115 = t8.*t10.*t111;
t116 = I_fx.*t2.*t7.*t111;
t118 = m_f.*r_r.*t4.*t7.*t110;
t119 = r_f.*t8.*t22.*t110;
t120 = r_f.*t8.*t22.*t111;
t126 = t10.*t39.*t40.*t110;
t131 = t8.*t22.*t39.*t41.*t110;
t145 = t46+t64+t138;
t146 = t45+t48+t49+t68+t82+t85+t102;
t153 = t124+t141;
t155 = t29+t127+t140;
t157 = t3.*t18.*t154;
t164 = t123+t130+t143;
t167 = t47+t125+t132+t144;
t179 = t28+t75+t80+t86+t88+t89+t90+t95+t96+t107+t108+t109+t134;
t121 = -t120;
t128 = t70+t115;
t133 = -t131;
t139 = -t10.*t39.*t41.*(t56-t115);
t147 = m_f.*r_r.*t11.*t145;
t148 = m_f.*t7.*t9.*t145;
t149 = e.*m_f.*t4.*t7.*t145;
t151 = t73+t106+t117+t118;
t152 = t3.*t8.*t18.*t146;
t156 = e.*m_f.*t155;
t158 = m_f.*r_r.*t2.*t7.*t153;
t160 = m_f.*r_r.*t4.*t7.*t155;
t161 = m_f.*t105.*t153;
t163 = m_f.*t110.*t155;
t166 = m_f.*r_r.*t2.*t7.*t164;
t169 = e.*m_f.*t167;
t170 = m_f.*t105.*t164;
t172 = m_f.*r_r.*t4.*t7.*t167;
t173 = m_f.*t110.*t167;
t181 = m_f.*t153.*t164;
t183 = t3.*t6.*t42.*t179;
t188 = m_f.*t155.*t167;
t150 = -t149;
t159 = -t158;
t162 = t119+t152;
t168 = t76+t121+t157;
t171 = -t170;
t182 = -t181;
t185 = -t183;
t165 = t33.*t39.*t41.*t162;
t174 = t33.*t39.*t41.*t168;
t191 = t113+t185;
t175 = t133+t165;
t176 = -i_fy.*t8.*t33.*(t131-t165);
t177 = -i_fy.*r_r.*t3.*t33.*(t131-t165);
t178 = i_fy.*r_r.*t3.*t33.*(t131-t165);
t180 = t139+t174;
t184 = i_fy.*t8.*t33.*(t174-t10.*t39.*t41.*(t56-t115));
t186 = i_fy.*r_r.*t3.*t33.*(t174-t10.*t39.*t41.*(t56-t115));
t192 = t33.*t39.*t40.*t191;
t198 = -i_fy.*(t174-t10.*t39.*t41.*(t56-t115)).*(t131-t165);
t187 = -t186;
t190 = t112+t184;
t193 = -t192;
t199 = t97+t159+t160+t178;
t200 = t36+t104+t161+t163+t176;
t203 = t25+t156+t198;
t189 = t63+t187;
t194 = t126+t193;
t195 = i_fy.*t8.*t33.*t194;
t196 = i_fy.*r_r.*t3.*t33.*t194;
t201 = -i_fy.*t194.*(t131-t165);
t202 = i_fy.*t194.*(t131-t165);
t204 = i_fy.*t194.*(t174-t10.*t39.*t41.*(t56-t115));
t197 = -t195;
t205 = -t204;
t207 = t30+t50+t147+t166+t172+t196;
t209 = t24+t59+t116+t150+t182+t188+t202;
t206 = t44+t169+t205;
t208 = t148+t171+t173+t197;
mt1 = [1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,m_r.*t17+m_r.*t21+m_f.*t105.^2+m_f.*t110.^2+i_fy.*t8.^2.*t34+m_f.*t19.*t21,t208,t200,t190,t151,0.0,0.0,0.0,0.0,t208,I_rz.*t17+I_ry.*t21+i_ry.*t21+I_fx.*t111.^2+i_fy.*t194.^2+m_f.*t145.^2+m_f.*t164.^2+m_f.*t167.^2+I_fz.*t17.*t19+m_r.*t13.*t21,t209,t206,t207,0.0,0.0,0.0,0.0,t200,t209,I_rx+I_fz.*t23+m_r.*t13+m_f.*t153.^2+m_f.*t155.^2+i_fy.*(t131-t165).^2+I_fx.*t15.*t19+m_f.*t12.*t16.*t19,t203,t199,0.0,0.0,0.0,0.0,t190,t206,t203,I_fz+m_f.*t12+i_fy.*(t174-t10.*t39.*t41.*(t56-t115)).^2,t189,0.0,0.0,0.0,0.0,t151,t207,t199,t189];
mt2 = [i_ry+m_r.*t14+m_f.*t14.*t23+m_f.*t14.*t15.*t19+m_f.*t14.*t16.*t19+i_fy.*t14.*t34.*1.0./t39.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,sigma];
Msharp = reshape([mt1,mt2],9,9);
