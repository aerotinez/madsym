function sys = pendulumStateSpace(p)
   %    states = [theta omega]
   %    inputs = [M_y]
   %    params = [b g l m t]
   M = pendulumSSMassMatrix(p);
   H = pendulumSSForcingMatrix(p);
   G = pendulumSSInputMatrix(p);
   sys = ss(H,G,eye(size(H)),0);

   sys.E = M;

   sys.Offsets.dx = [
      0;
      0;
   ];

   sys.Offsets.x = [
      0;
      0;
   ];

   sys.Offsets.u = [
      0;
   ];

   sys.StateName = [
      "theta";
      "omega";
   ];

   sys.InputName = [
      "M_y";
   ];

   sys.OutputName = [
      "theta";
      "omega";
   ];

end
