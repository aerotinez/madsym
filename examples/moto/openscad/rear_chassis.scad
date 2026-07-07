module rear_chassis(
    front_crown_radius = 300 - 215.9,
    front_radius = 215.9,
    rear_crown_radius = 325 - 215.9,
    rear_radius = 215.9,
    rake = 26.5,
    caster = 24,
    wheelbase = 1370,
    com_offset = 681.8797,
    com_height = 591.87
) {
    d_radius = (front_radius + front_crown_radius)
             - (rear_radius + rear_crown_radius);

    rear_chassis_length = wheelbase*cos(caster) + d_radius*sin(caster) - rake - front_crown_radius/2;

    fork_length = rear_radius + 2 * rear_crown_radius;
    head_tube_length = rear_chassis_length - fork_length;

    translate(-[com_offset,0,com_height - (rear_radius + rear_crown_radius)])
    rotate([0,90 - caster,0])
    {
        // Left fork lower
        translate([0, -1.5 * rear_crown_radius, 0])
            rotate([90, 0, 0])
                cylinder(
                    h = rear_crown_radius,
                    r = rear_crown_radius / 2,
                    center = true
                );

        // Left fork leg
        translate([
            0,
            -1.5 * rear_crown_radius,
            (rear_radius + rear_crown_radius) / 2
        ])
            cube([
                rear_crown_radius,
                rear_crown_radius,
                rear_radius + rear_crown_radius
            ], center = true);

        // Right fork lower
        translate([0, 1.5 * rear_crown_radius, 0])
            rotate([90, 0, 0])
                cylinder(
                    h = rear_crown_radius,
                    r = rear_crown_radius / 2,
                    center = true
                );

        // Right fork leg
        translate([
            0,
            1.5 * rear_crown_radius,
            (rear_radius + rear_crown_radius) / 2
        ])
            cube([
                rear_crown_radius,
                rear_crown_radius,
                rear_radius + rear_crown_radius
            ], center = true);

        // Fork crown
        translate([0, 0, rear_radius + 1.5 * rear_crown_radius])
            cube([
                rear_crown_radius,
                4 * rear_crown_radius,
                rear_crown_radius
            ], center = true);

        // Head tube
        translate([
            0,
            0,
            head_tube_length / 2 + rear_radius + 2 * rear_crown_radius
        ])
            cube([
                rear_crown_radius,
                front_crown_radius,
                head_tube_length
            ], center = true);
            
        // Joint
        translate([
                0,
                0,
                fork_length + head_tube_length + front_crown_radius/4
            ])
            cube([rear_crown_radius/2,front_crown_radius,front_crown_radius/2],center = true);
        
        translate([
                0,
                0,
                fork_length + head_tube_length + front_crown_radius/2
            ])
            rotate([0,90,0])
                cylinder(h = rear_crown_radius/2, r = front_crown_radius/2 ,center = true);
    }
}

// rotate([0,90-24,0])
rear_chassis();