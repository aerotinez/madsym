function res = ne(qa,qb)
    arguments
        qa (1,1) GeneralizedCoordinates;
        qb (1,1) GeneralizedCoordinates;
    end
    res = ~eq(qa,qb);
end