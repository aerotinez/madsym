function R = Rx(a)
    arguments
        a (1,1);
    end
    c = cos(a);
    s = sin(a);

    R = [
        1,0,0;
        0,c,-s;
        0,s,c
        ];
end

