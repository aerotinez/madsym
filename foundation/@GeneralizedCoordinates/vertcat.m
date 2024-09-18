function q = vertcat(qa,qb)
    arguments
        qa (1,1) GeneralizedCoordinates;
        qb (1,1) GeneralizedCoordinates;
    end

    all= [
        qa.All;
        qb.All
        ];

    dep = [
        qa.Dependent;
        qb.Dependent
        ];

    trim = [
        qa.Trim;
        qb.Trim
        ];

    trim_rate = [
        qa.TrimRate;
        qb.TrimRate
        ];

    q = GeneralizedCoordinates(all,dep,trim,trim_rate);
end