function x_dot = motorcycleKinematicsPrediction(x,u,p)
    arguments
        x (5,1) double;
        u (3,1) double;
        p (9,1) double;
    end
    M = motorcycleKinematicsMassMatrix(x,p);
    Minv = M\eye(size(M));
    J = Minv(:,1:3);
    x_dot = J*u;
    x_dot = x_dot(1:5,:);
end
