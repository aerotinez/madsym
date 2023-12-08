function [A,B] = stateSpace(linearized_equations)
arguments
    linearized_equations (1,1) LinearizedEquations
end
P = linearized_equations.PermutationMatrix;
M = linearized_equations.MassMatrix;
H = linearized_equations.ForcingMatrix;
G = linearized_equations.InputMatrix;
A = simplify(expand(P.'*(M\H)));
B = simplify(expand(P.'*(M\G)));