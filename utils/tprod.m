function J = tprod(T,x)
    J = sym(zeros(size(T,1),size(T,3)));
    for k = 1:size(T,3)
        J(:,k) = T(:,:,k)*x;
    end
end