function saveThesisFig(fig, name)
    arguments
        fig (1,1) matlab.ui.Figure
        name (1,1) string
    end

    % A4 setup
    page_width_cm = 21.0;
    margin_cm = 4.0;
    usable_width_cm = page_width_cm - 2 * margin_cm;

    % Screen DPI and figure aspect
    dpi = get(0, 'ScreenPixelsPerInch');
    fig_px = fig.Position;
    fig_cm = fig_px(3:4) * 2.54 / dpi;
    aspect_ratio = fig_cm(2) / fig_cm(1);

    % Detect tiledlayout and compute padding
    hasTiledLayout = any(arrayfun(@(c) isa(c, 'matlab.graphics.layout.TiledChartLayout'), fig.Children));
    vertical_padding_cm = 0;
    if hasTiledLayout
        tiled = findobj(fig.Children, 'Type', 'tiledlayout');
        if ~isempty(tiled)
            rows = tiled.GridSize(1);
            has_titles = any(arrayfun(@(a) ~isempty(a.Title.String), findall(fig, 'Type', 'Axes')));
            if has_titles
                vertical_padding_cm = rows * 0.8;
            else
                vertical_padding_cm = rows * 0.4;
            end
        else
            vertical_padding_cm = 1.5;
        end
    end

    % Compute height and resize
    height_cm = usable_width_cm * aspect_ratio + vertical_padding_cm;
    set(fig, 'Units', 'centimeters');
    set(fig, 'Position', [1 1 usable_width_cm height_cm]);

    % Redraw and export
    drawnow;
    exportgraphics(fig, name + ".eps", ...
        'ContentType', 'vector', ...
        'Resolution', 96, ...
        'BackgroundColor', 'white');
end
