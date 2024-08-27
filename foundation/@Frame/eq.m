function tf = eq(Na,Nb)
    arguments
        Na (1,1) Frame;
        Nb (1,1) Frame;
    end
    tf = isequal(Na.dcm(),Nb.dcm());
end