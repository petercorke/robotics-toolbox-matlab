R = 0.02;
RR = 1+4*R;
axle = 6*R;
rotate_extrude(convexity = 10, $fn = 100)
    translate([RR, 0, 0])
    circle(r =R, $fn = 100);

rotate([0, 90, 0]) {
    translate([0, 0, RR+axle/2])
        cylinder(h=axle, r=R, center=true);
    translate([0, 0, -RR-axle/2])
        cylinder(h=axle, r=R, center=true);
}