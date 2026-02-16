function introduce(obj,alpha_f)
    arguments
        obj (1,:) cell;
        alpha_f (1,1) double {mustBeInRange(alpha_f, 0, 1)} = 1;
    end

    fps = 60;
    ts = 1/fps;
    tf = 0.25;

    ns = tf / ts;

    a = linspace(0,1,ns);

    for k = 1:ns
        for i = 1:numel(obj)
            setAlpha(obj{i},a(k));
        end
        pause(ts);
    end

    if alpha_f ~= 1
        a = fliplr(linspace(alpha_f,1,floor(ns/2)));

        pause(0.25);

        for k = 1:floor(ns/2)
            for i = 1:numel(obj)
                setAlpha(obj{i},a(k));
            end
            pause(ts);
        end
    end
end