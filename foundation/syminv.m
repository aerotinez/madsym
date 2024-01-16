function B = syminv(A)
if isempty(A)
    B = A;
    return
end
if isequal(size(A),[1,1])
    B = 1/A;
    return
end
idx = has(A,[symvar(A),1,-1]);
inv_map = idx.*sym('a_',size(A));
B = subs(inv(inv_map),inv_map(idx),A(idx));