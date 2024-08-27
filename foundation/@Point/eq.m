function tf = eq(Pa,Pb)
    arguments
        Pa (1,1) Point;
        Pb (1,1) Point;
    end
    tf = isequal(Pa.posFrom(),Pb.posFrom());
end