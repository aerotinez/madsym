setmadsympath();
close("all"); clear; clc;

u = [-ones(1,100),ones(1,300),-ones(1,200),-ones(1,200),ones(1,300),-ones(1,100)];
v = cumsum(u);
p = cumsum(v);

hold("on");
plot(u./max(u));
plot(v./max(v));
plot(p./max(p));
hold("off");