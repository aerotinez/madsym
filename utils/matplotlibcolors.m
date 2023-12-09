function c = matplotlibcolors()
% names = [
%     "blue";
%     "orange";
%     "green";
%     "red";
%     "purple";
%     "brown";
%     "pink";
%     "gray";
%     "olive";
%     "cyan"
%     ];

hex_vals = [
    '1f77b4';
    'ff7f0e';
    '2ca02c';
    'd62728';
    '9467bd';
    '8c564b';
    'e377c2';
    '7f7f7f';
    'bcbd22';
    '17becf'
    ];

r = hex_vals(:,1:2);
g = hex_vals(:,3:4);
b = hex_vals(:,5:6);
% nrgb = num2cell(cell2mat(cellfun(@hex2dec,{r,g,b},'uniform',0))./255,2);
% c = cell2struct(nrgb,names,1);
c = cell2mat(cellfun(@hex2dec,{r,g,b},'uniform',0))./255;