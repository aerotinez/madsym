module Base(
    breadth = 40E-03, 
    depth = 30E-03, 
    height=20E-03, 
    scale_top=0.6, 
    center=true
    ) {
    linear_extrude(height=height, scale=scale_top, center=center)
        square([breadth, depth], center=true);
}



Base();