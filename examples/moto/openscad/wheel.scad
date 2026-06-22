module tire(radius, crown_radius) {
    difference() {
        rotate([90, 0, 0])
            rotate_extrude()
                translate([radius, 0, 0])
                    circle(r = crown_radius);

        rotate([90, 0, 0])
            translate([0, 0, -crown_radius])
                cylinder(d = 2 * radius, h = 2 * crown_radius);
    }
}

module axle(crown_radius) {
    axle_radius = crown_radius / 2;

    rotate([90, 0, 0])
        translate([0, 0, -crown_radius])
            cylinder(d = 2 * axle_radius, h = 2 * crown_radius);
}

module spoke(inner_r, outer_r, thickness, angle) {
    rotate_extrude(angle = angle)
        translate([inner_r, -thickness / 2, 0])
            square([outer_r - inner_r, thickness]);
}

module wheel(radius, crown_radius, spoke_count = 6) {
    axle_radius = crown_radius / 2;
    spoke_angle = 360 / spoke_count * 0.35;
    spoke_thickness = crown_radius / 2;

    tire(radius, crown_radius);
    axle(crown_radius);

    rotate([90, 0, 0])
        for (i = [0 : spoke_count - 1]) {
            rotate([0, 0, i * 360 / spoke_count - spoke_angle / 2])
                spoke(
                    inner_r = axle_radius,
                    outer_r = radius,
                    thickness = spoke_thickness,
                    angle = spoke_angle
                );
        }
}