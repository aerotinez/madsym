function B = syminv(A)
if isempty(A)
    B = A;
    return
end
if isequal(size(A),[1,1])
    B = 1/A;
    return
end
idx = has(A,[symvar(A),1,-1,2,-2]);
inv_map = logical(arrayfun(@nnz,A)).*sym('a_',size(A));
B = subs(inv(inv_map),inv_map(idx),A(idx));
% B = subs(inv_map\eye(size(inv_map)),inv_map(idx),A(idx));