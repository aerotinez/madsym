close("all"); clear; clc;
setmadsympath;

bst = bikeSimV102Tire;
par = 'C:\Users\Public\Documents\BikeSim2017.1_Data\Tires\Tire\Tire_374bb211-df93-40bd-bbc9-7b64c92aa42c.par';
pac = 'C:\Users\Public\Documents\BikeSim2017.1_Data\Generic\Example_Front_v102.par';
bst.import(par, pac);
plot(bst);

