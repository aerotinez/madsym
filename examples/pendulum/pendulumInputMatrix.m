function out1 = pendulumInputMatrix(in1,in2)
%pendulumInputMatrix
%    OUT1 = pendulumInputMatrix(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    05-Jun-2024 14:37:14

%    states = [theta omega]
%    params = [b g l m]
%
l = in2(3,:);
m = in2(4,:);
out1 = [0.0;1.0./l.^2./m];