function vCf = frontContactVelocity(in1,in2)
%frontContactVelocity
%    vCf = frontContactVelocity(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    02-Jul-2024 18:02:31

delta = in1(4,:);
l_f = in2(3,:);
omega_z = in1(7,:);
v_x = in1(5,:);
v_y = in1(6,:);
t2 = cos(delta);
t3 = sin(delta);
vCf = [t2.*v_x+t3.*v_y+l_f.*omega_z.*t3;-t3.*v_x+t2.*v_y+l_f.*omega_z.*t2];
