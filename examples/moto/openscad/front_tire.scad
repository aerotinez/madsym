use <wheel.scad>

front_radius = 215.9;
front_crown_radius = 300 - front_radius;
spoke_count = 6;

$fn = 72;

wheel(front_radius, front_crown_radius, spoke_count);