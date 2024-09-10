function sdet = symdet(A)
    arguments
        A sym {mustBeSquareMatrix};
    end
    switch size(A,1)
        case 1
            sdet = A;
            return
        case 2
            sdet = det2(A);
            return
        case 3
            sdet = det3(A);
            return
        case 4
            sdet = simplify(expand(det(A)));
            return
        otherwise
            sdet = zeros(1,"sym");
            for k = 1:size(A,1)
                C = A(2:end,[1:k - 1, k + 1:end]);
                sdetn = symdet(C);
                S = (-1)^(1 + k);
                sdet = sdet + S.*A(1,k)*sdetn;
            end
    end
end

function mustBeSquareMatrix(A)
    if size(A,1) ~= size(A,2)
        error("Input matrix must be square");
    end
end

function sdet = det2(A)
    a = A(1,1);
    b = A(1,2);
    c = A(2,1);
    d = A(2,2);
    sdet = simplify(expand(a*d - b*c));
end

function sdet = det3(A)
    a = A(1,1);
    b = A(1,2);
    c = A(1,3);
    idx = [2,3];
    Ca = det2(A(idx,[2,3]));
    Cb = det2(A(idx,[1,3]));
    Cc = det2(A(idx,[1,2]));
    sdet = simplify(expand(a.*Ca - b.*Cb + c.*Cc));
end