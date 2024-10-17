function f0 = smallang(f,angs)
    arguments
        f sym;
        angs (:,1) DynamicVariable;
    end
    sinx = sin(angs.state);
    cosx = cos(angs.state);
    g = @(k)prod(nchoosek(sinx,k),2);
    prods = cell2sym(arrayfun(g,fliplr(2:numel(sinx)),'uniform',0).');

    ovars = [
        prods;
        sinx;
        cosx
        ];

    nvars = [
        zeros(size(prods),"sym");
        angs.state;
        ones(size(angs.state),"sym")
        ];

    f0 = simplify(expand(subs(f,ovars,nvars)));
end