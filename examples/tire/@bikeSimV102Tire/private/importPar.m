function importPar(obj,par)
    arguments
        obj (1,1) bikeSimV102Tire
        par (1,1) string
    end

    fid = fopen(par,"r");
    if fid == -1
        
        error("bikeSimV102Tire:importPar:FileNotFound", ...
              "Could not open PAR file: %s", par);
    end

    cleanup = onCleanup(@() fclose(fid));

    propNames = properties(obj);

    while ~feof(fid)
        line = strtrim(fgetl(fid));

        if line == "" || startsWith(line,"#")
            continue
        end

        tokens = split(line);
        if numel(tokens) < 2
            continue
        end

        name = tokens(1);
        value = str2double(tokens(2));

        if isnan(value)
            continue
        end

        idx = strcmp(propNames,name);
        if any(idx)
            obj.(propNames{idx}) = value;
        end
    end

    obj.SET_THICKNESS_SGUI = 1E-03*obj.SET_THICKNESS_SGUI;
    obj.RRE = 1E-03*obj.RRE;
    obj.KT = 1E03*obj.KT;
    obj.RAD_TC = 1E-03*obj.RAD_TC;
    obj.L_RELAX_X = 1E-03*obj.L_RELAX_X;
    obj.L_RELAX_Y = 1E-03*obj.L_RELAX_Y;
    obj.VLOW_ALPHA = obj.VLOW_ALPHA/3.6;
    obj.RR_V = 3.6*obj.RR_V;
end