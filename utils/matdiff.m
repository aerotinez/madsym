function DA = matdiff(A,x)
    DA = zeros([size(A),numel(x)],'sym');
    for k = 1:size(DA,1)
        DA(k,:,:) = reshape(jacobian(A(k,:),x),1,size(DA,2),[]);
    end
end
