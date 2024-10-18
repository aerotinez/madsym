function W = minus(Wa,Wb)
    arguments
        Wa (1,1) Wrench;
        Wb (1,1) Wrench;
    end
    W = Wrench(Wa.vector() - Wb.vector(),Pose());
end