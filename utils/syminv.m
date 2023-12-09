function B = syminv(A)
idx = has(A,[symvar(A),1,-1]);
inv_map = idx.*sym('a_',size(A));
B = subs(inv(inv_map),inv_map(idx),A(idx));