function res = eq(qa,qb)
    arguments
        qa (1,1) GeneralizedCoordinates;
        qb (1,1) GeneralizedCoordinates;
    end
    f = @(x)[
        x.All;
        x.Dependent;
        x.Trim;
        x.TrimRate
        ];

    res = false;
    if isequal(f(qa),f(qb))
        res = true;
    end
end