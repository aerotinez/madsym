function f0 = subsTrim(f,x)
    arguments
        f sym;
        x (:,1) DynamicVariable;
    end
    ovals = [
        x.rate;
        x.state
        ];

    nvals = [
        [x.TrimRate].';
        [x.TrimState].'
    ];
    
    f0 = subs(f,ovals,nvals);
end