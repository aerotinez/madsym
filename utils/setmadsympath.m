function setmadsympath()
fpath = matlab.desktop.editor.getActiveFilename;
if ispc
    [~,idx] = regexp(fpath,"\");
else
    [~,idx] = regexp(fpath,"/");
end
cpath = fpath(1:idx(end));
cd(cpath);