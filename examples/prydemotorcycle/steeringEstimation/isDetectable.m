function out = isDetectable(A, C)
%ISDETECTABLE  Check detectability of (A,C) in continuous-time.
%
%   out = ISDETECTABLE(A, C) returns TRUE if (A,C) is
%   detectable (in continuous-time). Otherwise, it returns FALSE.
%
%   A pair (A,C) is detectable if every eigenvalue of A with real(lambda) >= 0
%   is observable. Equivalently, for each such lambda, the matrix
%       [lambda*I - A; C]
%   must have full column rank (i.e., rank == n).
%
%   Example:
%       A = [0 1; -2 -3];
%       C = [1 0];
%       out = isDetectable(A, C)

    % System dimension
    n = size(A, 1);
    
    % Compute the eigenvalues of A
    lambdaVals = eig(A);
    
    % Assume detectability unless a counterexample is found
    out = true(size(A,1),1);
    
    % Check each eigenvalue
    for k = 1:length(lambdaVals)
        lam = lambdaVals(k);
        
        % We only test eigenvalues with Re(lambda) >= 0
        if real(lam) >= 0
            % Form the matrix [lam*I - A; C]
            M = [lam*eye(n) - A; C];
            
            % Check rank
            if rank(M) < n
                % If rank < n, then (A,C) is not detectable
                out(k) = false;
            end
        end
    end
end
