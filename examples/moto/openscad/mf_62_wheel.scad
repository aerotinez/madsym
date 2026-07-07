module tire(
    UNLOADED_RADIUS = 325.0,
    WIDTH = 190.0,
    ASPECT_RATIO = 0.55,
    RIM_RADIUS = 215.9,
    RIM_WIDTH = 152.4,
    MC_CONTOUR_A = 1.2781,
    MC_CONTOUR_B = 2.9277
) {
    rotate([90, 0, 0])
        rotate_extrude()
            translate([UNLOADED_RADIUS, 0, 0])
                resize(2*WIDTH*[MC_CONTOUR_B,MC_CONTOUR_A])circle(d = 2 * WIDTH);
}

tire();