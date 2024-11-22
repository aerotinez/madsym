function sys = quadcopterStateSpace(p)
   %    states = [psi theta phi p_x p_y p_z omega_x omega_y omega_z v_x v_y v_z]
   %    inputs = [tau_x tau_y tau_z f_z]
   %    params = [I_xx I_yy I_zz g m]
   P = quadcopterPermutationMatrix();
   M = quadcopterMassMatrix(p);
   H = quadcopterForcingMatrix(p);
   G = quadcopterInputMatrix(p);
   A = P*(M\H);
   B = P*(M\G);
   sys = ss(A,B,eye(size(A)),0);
end
