use <wheel.scad>

rear_radius = 215.9;
rear_crown_radius = 325 - rear_radius;
spoke_count = 6;

$fn = 72;

wheel(rear_radius, rear_crown_radius, spoke_count);