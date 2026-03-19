close("all"); clear; clc;
setmadsympath();

dir="C:\Users\Public\Documents\BikeSim2017.1_Data\Generic\";
fig_front=pacejkaSideForcePlot(dir+"Example_Front_v102.par","Front tire");
set(fig_front,"Position",[100,480,1280,240]);

fig_rear=pacejkaSideForcePlot(dir+"Example_Rear_v102.par","Rear tire");

%% Plotting functions

function fig=pacejkaSideForcePlot(file_name,title_str)
    pars=readPacejka(file_name);
    m=matlabColors();
    
    ns=1000;
    slip=deg2rad(linspace(-10,10,ns));

    camber_max=50;
    camber=deg2rad(linspace(-camber_max,camber_max,ns));
    fz=linspace(0,1.5E03,ns);
    
    % Create tiled layout
    fig=figure('Position',[100,100,1280,240]);
    
    % Set global font defaults
    set(fig,'DefaultAxesFontSize',12);
    set(fig,'DefaultTextFontSize',12);
    
    t=tiledlayout(fig,1,3,'TileSpacing','compact','Padding','compact');
    sgtitle(title_str+" side force: Magic formula","FontSize",12);
    
    % -------- 1) Slip vs camber --------
    [X,Y]=meshgrid(slip,camber);
    Z=pacejkaSideForce(X,Y,fz(end),pars);
    
    axe=nexttile(t,1);

    Zplot=Z./1E03;
    surf(axe,rad2deg(X),rad2deg(Y),Zplot, ...
        'EdgeColor','none', ...
        'FaceColor',m.blue, ...
        'FaceLighting','gouraud');
    
    view(axe,3);
    box(axe,'on'); grid(axe,'on'); axis(axe,'tight');
    title(axe,"Slip vs camber vs side force (Z = " + fz(end)/1E03 + " kN)", ...
        'FontSize',12);
    xlabel(axe,'Slip angle (deg)','FontSize',12);
    ylabel(axe,'Camber (deg)','FontSize',12);
    zlabel(axe,'Force (kN)','FontSize',12);
    camlight(axe,60,45,"local");
    camproj(axe,'perspective');
    lims = max(abs(Zplot(ns/2,:))).*[-1,1];
    
    % -------- 2) Slip vs vertical force --------
    fy=pacejkaSideForce(slip,0*camber,fz.',pars);
    fyl=pacejkaSideForce(slip,0*camber,fz(end),pars,"Type","Linearized");
    
    X=rad2deg(repmat(slip,ns,1));
    Y=fy./1E03;
    Z=repmat(fz,ns,1)./1E03;
    
    axe=nexttile(t,2);
    hold(axe,'on');
    surf(axe,X.',Y.',Z,'EdgeColor','none');
    plot3(rad2deg(slip),fyl./1E03,fz(end)*ones(ns,1), ...
        'Color',m.orange, ...
        'LineWidth',2);
    hold(axe,'off');
    view(axe,0,90);
    box(axe,'on'); axis(axe,'tight'); grid(axe,'on');
    title(axe,'Slip vs normal force vs side force','FontSize',12);
    xlabel(axe,'Slip angle (deg)','FontSize',12);
    ylim(axe,lims);
    
    % -------- 3) Camber vs vertical force --------
    fy=pacejkaSideForce(0*slip,camber,fz.',pars); 
    fyl=pacejkaSideForce(0*slip,camber,fz(end),pars,"Type","Linearized");

    X=rad2deg(repmat(camber,ns,1));
    Y=fy./1E03;
    Z=repmat(fz,ns,1)./1E03;
    
    axe=nexttile(t,3);
    hold(axe,'on');
    surf(axe,X.',Y.',Z,'EdgeColor','none',"DisplayName","nonlinear");
    plot3(rad2deg(camber),fyl./1E03,fz(end)*ones(ns,1), ...
        'Color',m.orange, ...
        'LineWidth',2, ...
        "DisplayName","linearized (Z = " + fz(end)/1E03 + " kN)");
    hold(axe,'off');
    view(axe,0,90);
    box(axe,'on'); axis(axe,'tight'); grid(axe,'on');
    title(axe,'Camber vs normal force vs side force','FontSize',12);
    xlabel(axe,'Camber angle (deg)','FontSize',12);
    xticks(axe,-camber_max:camber_max/2:camber_max);
    ylim(axe,lims);
    
    % Only one colorbar (attached to last tile)
    cb=colorbar(axe);
    cb.FontSize=12;
    cb.Label.String="Normal force (kN)";
    cb.Label.FontSize=12;

    lgd = legend("show", ...
        "FontSize",10, ...
        "Location","SouthEast", ...
        "Orientation","vertical");

    % lgd.ItemTokenSize = 12;
end

%% Helper functions

function fy=pacejkaSideForce(slip,camber,fz,pars,options)

    arguments
        slip double
        camber double
        fz double
        pars struct
        options.Type string {mustBeMember(options.Type,["Full","Linearized"])} = "Full"
    end

    epsy=1e-12;

    fz0=getPar(pars,'P_BIKE_FZ0',0);
    if abs(fz0)<epsy
        eID='pacejkaLateralForce:InvalidFZ0';
        error(eID,'P_BIKE_FZ0 is missing or zero in the parameter struct.');
    end
    dfz=(fz-fz0)./fz0;

    % Shape factors
    pCy1=getPar(pars,'P_BIKE_PCY1',0);
    pCy2=getPar(pars,'P_BIKE_PCY2',0);

    % Peak factor
    pDy1=getPar(pars,'P_BIKE_PDY1',0);
    pDy2=getPar(pars,'P_BIKE_PDY2',0);
    pDy3=getPar(pars,'P_BIKE_PDY3',0);

    % Curvature factor
    pEy1=getPar(pars,'P_BIKE_PEY1',0);
    pEy2=getPar(pars,'P_BIKE_PEY2',0);
    pEy3=getPar(pars,'P_BIKE_PEY3',0);
    pEy4=getPar(pars,'P_BIKE_PEY4',0);
    pEy5=getPar(pars,'P_BIKE_PEY5',0);

    % Cornering stiffness
    pKy1=getPar(pars,'P_BIKE_PKY1',0);
    pKy2=getPar(pars,'P_BIKE_PKY2',0);
    pKy3=getPar(pars,'P_BIKE_PKY3',0);
    pKy4=getPar(pars,'P_BIKE_PKY4',0);
    pKy5=getPar(pars,'P_BIKE_PKY5',0);
    pKy6=getPar(pars,'P_BIKE_PKY6',0);
    pKy7=getPar(pars,'P_BIKE_PKY7',0);

    % Horizontal shift
    pHy1=getPar(pars,'P_BIKE_PHY1',0);

    % Shifted slip angle
    SHy=pHy1;
    alpha_y=slip+SHy;

    % Main shape factor
    Cy=pCy1;

    % Peak factor
    mu_y=pDy1.*exp(pDy2.*dfz)./(1+pDy3.*camber.^2);
    Dy=mu_y.*fz;

    % Curvature factor
    Ey_term1=pEy1+pEy2.*camber.^2;
    Ey_term2=(pEy3+pEy4.*camber).*sign(alpha_y);
    Ey=Ey_term1+Ey_term2;

    % Cornering stiffness
    denomKya0=(pKy3+pKy4.*camber.^2).*fz0;
    atan_term=atan(fz./(denomKya0+epsy));
    Kya0=pKy1.*fz0.*sin(pKy2.*atan_term);
    Kya=Kya0./(1+pKy5.*camber.^2);

    % Slip stiffness factor
    By=Kya./(Cy.*Dy+epsy);

    % Camber term coefficients
    Cgamma=pCy2;
    Kyg=(pKy6+pKy7.*dfz).*fz;
    Eg=pEy5;
    Bgamma=Kyg./(Cgamma.*Dy+epsy);

    if strcmp(options.Type,"Linearized")
        fy=-(Kya.*alpha_y+Kyg.*camber);
        return
    end

    % Full nonlinear slip contribution
    By_alpha=By.*alpha_y;
    atan_By_alpha=atan(By_alpha);
    Fy_alpha_inner=By_alpha-Ey.*(By_alpha-atan_By_alpha);
    Fy_alpha=Dy.*sin(Cy.*atan(Fy_alpha_inner));

    % Full nonlinear camber contribution
    Bgamma_camber=Bgamma.*camber;
    Fy_gamma_inner=Bgamma_camber-Eg.*(Bgamma_camber-atan(Bgamma_camber));
    Fy_gamma=Dy.*sin(Cgamma.*atan(Fy_gamma_inner));

    fy=-(Fy_alpha+Fy_gamma);
end

function value=getPar(pars,name,defaultValue)
    if isfield(pars,name)
        value=pars.(name);
    else
        value=defaultValue;
    end
end

function pars=readPacejka(filename)
   arguments
        filename (1,1) string
    end

    if ~isfile(filename)
        eID='readPacBikeParFile:FileNotFound';
        error(eID,'File not found: %s',filename);
    end

    fid=fopen(filename,'r');
    if fid==-1
        eID='readPacBikeParFile:OpenFailed';
        error(eID,'Could not open file: %s',filename);
    end

    cleaner=onCleanup(@() fclose(fid));

    pars=struct();
    inBlock=false;

    while ~feof(fid)
        line=fgetl(fid);

        if ~ischar(line)
            continue
        end

        line=strtrim(line);

        if line==""
            continue
        end

        if startsWith(line,'#')||startsWith(line,'%')||startsWith(line,'//')
            continue
        end

        if strcmp(line,'BEGIN_PAC_BIKE_DATA')
            inBlock=true;
            continue
        elseif strcmp(line,'END_PAC_BIKE_DATA')
            inBlock=false;
            continue
        elseif strcmp(line,'Parsfile')||strcmp(line,'END')
            continue
        end

        if ~inBlock
            continue
        end

        expr='^([A-Za-z]\w*)\s+([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)$';
        tokens=regexp(line,expr,'tokens','once');

        if isempty(tokens)
            continue
        end

        name=tokens{1};
        value=str2double(tokens{2});

        if isnan(value)
            eID='readPacBikeParFile:InvalidValue';
            warning(eID,'Skipping line with invalid numeric value: %s',line);
            continue
        end

        fieldName=matlab.lang.makeValidName(name);
        pars.(fieldName)=value;
    end
end