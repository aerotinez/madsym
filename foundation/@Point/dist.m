function d = dist(Pa,Pb)
    arguments
        Pa (1,1) Point;
        Pb (1,1) Point;
    end
    r = Pa.posFrom() - Pb.posFrom();
    d = sqrt(r.'*r);
end