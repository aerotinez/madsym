function R = Rz(a)
    arguments
        a (1,1);
    end
    c = cos(a);
    s = sin(a);

    R = [
        c,-s,0;
        s,c,0;
        0,0,1
        ];
end