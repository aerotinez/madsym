function f0 = smallang(f, angs)
    arguments
        f sym
        angs (:,1) sym
    end

    x = angs;

    f0 = rewrite(f, 'sincos');
    f0 = expand(f0);

    % First-order trig substitutions
    f0 = subs(f0,cos(x),sym(ones(size(x))));
    f0 = subs(f0,sin(x),x);

    % Expand again so newly created products are visible
    f0 = expand(f0);

    % Remove all quadratic and higher terms in the chosen small angles
    prods = triu(x*x.',1);

    x0 = [
        nonzeros(prods(:));
        x.^2;
        x.^3;
        x.^4;
        x.^5
        ];

    f0 = subs(f0,x0,zeros(size(x0)));
    f0 = simplify(collect(f0,x));
    f0 = subs(f0,x0,zeros(size(x0)));
end