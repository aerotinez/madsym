module front_chassis(
    front_crown_radius = 50,
    front_radius = 232,
    rear_crown_radius = 70,
    rear_radius = 227,
    rake = 26.5,
    caster = 24,
    wheelbase = 1370
) {
    d_radius = (front_radius + front_crown_radius)
             - (rear_radius + rear_crown_radius);

    fork_length = front_radius + 2 * front_crown_radius;
    front_chassis_length = wheelbase*sin(caster) - d_radius*cos(caster);
    head_tube_length = front_chassis_length - fork_length + rear_crown_radius/2;

    rotate([0,-caster,0]) {
        // Left fork lower
        translate([0, -1.5 * front_crown_radius, 0])
            rotate([90, 0, 0])
                cylinder(
                    h = front_crown_radius,
                    r = front_crown_radius / 2,
                    center = true
                );

        // Left fork leg
        translate([
            0,
            -1.5 * front_crown_radius,
            (front_radius + front_crown_radius) / 2
        ])
            cube([
                front_crown_radius,
                front_crown_radius,
                front_radius + front_crown_radius
            ], center = true);

        // Right fork lower
        translate([0, 1.5 * front_crown_radius, 0])
            rotate([90, 0, 0])
                cylinder(
                    h = front_crown_radius,
                    r = front_crown_radius / 2,
                    center = true
                );

        // Right fork leg
        translate([
            0,
            1.5 * front_crown_radius,
            (front_radius + front_crown_radius) / 2
        ])
            cube([
                front_crown_radius,
                front_crown_radius,
                front_radius + front_crown_radius
            ], center = true);

        // Fork crown
        translate([0, 0, front_radius + 1.5 * front_crown_radius])
            cube([
                front_crown_radius,
                4 * front_crown_radius,
                front_crown_radius
            ], center = true);

        // Head tube
        translate([
            0,
            0,
            head_tube_length / 2 + front_radius + 2 * front_crown_radius
        ])
            cube([
                front_crown_radius,
                front_crown_radius,
                head_tube_length
            ], center = true);
            
        // knuckle
        translate([
                -(rake - front_crown_radius)/2,
                0,
                fork_length + head_tube_length - rear_crown_radius/2
            ])
                cube([rake,front_crown_radius,rear_crown_radius],center = true);
        
        translate([
                -(rake)/2,
                0,
                fork_length + head_tube_length - rear_crown_radius/8
            ])
                cube([rake,front_crown_radius,rear_crown_radius/4],center = true);
                
        translate([
                -(rake)/2,
                0,
                fork_length + head_tube_length - rear_crown_radius + rear_crown_radius/8
            ])
                cube([rake,front_crown_radius,rear_crown_radius/4],center = true);
            
        // Steering joint
        difference() {
            translate([
                -rake,
                0,
                fork_length + head_tube_length - rear_crown_radius/2
            ])
                cylinder(h = rear_crown_radius, r = front_crown_radius/2 ,center = true);
                
            translate([
                -rake,
                0,
                fork_length + head_tube_length - rear_crown_radius/2
            ])
                cylinder(h = rear_crown_radius/2, r = front_crown_radius ,center = true);
        }
    }
}

//rotate([0,-24,0])
front_chassis();