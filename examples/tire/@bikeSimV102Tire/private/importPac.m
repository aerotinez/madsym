function importPac(obj,pac)
    arguments
        obj (1,1) bikeSimV102Tire
        pac (1,1) string
    end

    fid = fopen(pac,"r");
    if fid == -1
        error("bikeSimV102Tire:importPac:FileNotFound", ...
            "Could not open PAC file: %s", pac);
    end

    cleanup = onCleanup(@() fclose(fid));

    propNames = properties(obj);
    inPacData = false;

    while ~feof(fid)
        line = strtrim(fgetl(fid));

        if line == "" || startsWith(line,"#")
            continue
        end

        if strcmpi(line,"BEGIN_PAC_BIKE_DATA")
            inPacData = true;
            continue
        elseif strcmpi(line,"END_PAC_BIKE_DATA")
            break
        end

        if ~inPacData
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
end