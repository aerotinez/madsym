function out1 = steeringEstimationForcingVector(in1,in2,in3)
%steeringEstimationForcingVector
%    OUT1 = steeringEstimationForcingVector(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    23-Feb-2025 18:01:56

omega_x = in2(:,1);
omega_y = in2(:,2);
omega_z = in2(:,3);
out1 = [-omega_x;-omega_y;-omega_z;0.0];
end
