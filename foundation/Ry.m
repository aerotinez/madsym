function R = Ry(a)
    arguments
        a (1,1);
    end
    c = cos(a);
    s = sin(a);
    
    R = [
        c,0,s;
        0,1,0;
        -s,0,c
        ];
end