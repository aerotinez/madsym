use <wheel.scad>
use <front_chassis.scad>
use <rear_chassis.scad>

front_crown_radius = 50;
front_radius = 232;
rear_crown_radius = 70;
rear_radius = 227;
rake = 26.5;
caster = 24;
wheelbase = 1370;

translate([0,0,rear_radius + rear_crown_radius])
    wheel(radius = rear_radius, crown_radius = rear_crown_radius);
    
translate([wheelbase,0,front_radius + front_crown_radius])
    wheel(radius = front_radius, crown_radius = front_crown_radius);
    
translate([wheelbase,0,front_radius + front_crown_radius])
        front_chassis(rake=rake);
        
translate([0,0,rear_radius + rear_crown_radius])
        rear_chassis(rake=rake);